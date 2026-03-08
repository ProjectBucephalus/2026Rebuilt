package frc.robot.commands.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Pathfinding.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Conversions;
import static frc.robot.constants.Constants.SwerveConstants.*;

/** 
 * A drive command for following pre-planned paths 
 * @author 5985
 */
public class PathFollowDrive extends SwerveCommandBase 
{
  private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();    
  
  private final PIDController xController = new PIDController(driveKP, driveKI, driveKD);
  private final PIDController yController = new PIDController(driveKP, driveKI, driveKD);
  private final PIDController thetaController = new PIDController(rotationKP, rotationKI, rotationKD);
  {thetaController.enableContinuousInput(-Math.PI, Math.PI);} // Init block- runs when constructed, regardless of constructor used

  private final Supplier<SwerveDriveState> swerveStateSup;
  private final DoubleSupplier brakeSup;

  private final ArrayList<Pose2d> waypoints;
  private final ArrayList<Double> radiusPerSegment;

  private boolean onPath = false;
  private int currentWaypoint = 0;

  /**
   * Creates a new PathFollowDrive to follow the given sequence
   * @param s_Swerve       Swervedrive subsystem
   * @param swerveStateSup Swerve state supplier from Robot to avoid expensive calls to the swerve system
   * @param path           Predefined path for command to follow
   * @param brakeSup       Speed reduction to apply, [0..1]. Higher is slower
   */
  public PathFollowDrive(CommandSwerveDrivetrain s_Swerve, Supplier<SwerveDriveState> swerveStateSup, Path path, DoubleSupplier brakeSup)
  {
    super(s_Swerve, () -> Translation2d.kZero);
    this.swerveStateSup = swerveStateSup;
    this.brakeSup = brakeSup;

    this.waypoints = new ArrayList<>(path.sequence().length * 3 - 2);
    this.radiusPerSegment = new ArrayList<>(path.sequence().length);

    for (int i = 0; i < path.sequence().length - 1; i++) 
    {
      final var current = path.sequence()[i];
      final var next = path.sequence()[i + 1];

      // Find the distance between the current point and the next
      // If the input radius is greater than 1/3 the distance between points, use 1/3 for next step
      final double segmentLength = current.getDistance(next);
      final double waypointDist = Conversions.clamp(path.pointRadius(), 0, segmentLength / 3);
      final double lengthRatio = waypointDist / segmentLength;

      // Project additional waypoints using input radius to give a smoother path
      final var waypoint1 = current.interpolate(next, lengthRatio);
      final var waypoint2 = next.interpolate(current, lengthRatio);

      // Record distance at which to switch waypoints for this segment
      radiusPerSegment.add(waypointDist);
      // Add current waypoint and projected midpoints to path list
      waypoints.addAll
      (
        List.of
        (
          new Pose2d(current, path.heading()), 
          new Pose2d(waypoint1, path.heading()), 
          new Pose2d(waypoint2, path.heading())
        )
      );
    }

    // Final waypoint does not trigger until the robot arives at it
    radiusPerSegment.add(0.0);
    waypoints.add(new Pose2d(path.sequence()[path.sequence().length - 1], path.heading()));
  }

  /**
   * Creates a new PathFollowDrive to pathfind to the given pose
   * @param s_Swerve        Swervedrive subsystem
   * @param swerveStateSup  Swerve state supplier from Robot to avoid expensive calls to the swerve system
   * @param path            Target pose for the command to drive to
   */
  public PathFollowDrive(CommandSwerveDrivetrain s_Swerve, Supplier<SwerveDriveState> swerveStateSup, Pose2d target)
  {
    super(s_Swerve, () -> Translation2d.kZero);
    this.swerveStateSup = swerveStateSup;
    brakeSup = () -> 0;

    this.waypoints = new ArrayList<>();
    this.radiusPerSegment = new ArrayList<>();

    waypoints.add(target);
    radiusPerSegment.add(0.0);
  }

  @Override
  public void initDriveConstraints() 
  {
    currentWaypoint = 0;
    onPath = false;
  }

  @Override
  public void execute() 
  {
    robotPose = swerveStateSup.get().Pose;

    // If the robot is close to the path, follow one point ahead to give smoother cornering
    final var targetIndex = Math.min(onPath ? currentWaypoint + 1 : currentWaypoint, waypoints.size() - 1);
    final var targetPose = waypoints.get(targetIndex);
    
    s_Swerve.setControl(driveRequest.withSpeeds(calculateDrivePID(targetPose, robotPose)));
        
    // Switch to next waypoint when within the given distance of the current one
    final var currentSegment = Math.floorDiv(currentWaypoint, 3);
    final double targetDist = radiusPerSegment.get(currentSegment);

    if (Conversions.nearTranslation(robotPose.getTranslation(), targetPose.getTranslation(), targetDist)) 
    {
      currentWaypoint = Math.min(currentWaypoint + 1, waypoints.size());
      onPath = true;
    }
  }

  @Override
  public boolean isFinished() 
  {
    // Finish when robot is at the final waypoint
    return Conversions.atPose(robotPose, waypoints.get(waypoints.size()-1));
  }

  /**
   * Calculates the required drive input to drive to a pose using a PID control loop, accounting for geofencing
   * 
   * @param target Current target pose to drive towards
   * @param pose Current robot pose
   * @return Chassis speeds, m/s, m/s, rad/s
   */
  public ChassisSpeeds calculateDrivePID(Pose2d target, Pose2d pose)
  {
    final double brake = 1 - brakeSup.getAsDouble();
    final var robotPos = pose.getTranslation();
    final var targetPos = target.getTranslation();

    double speedX = xController.calculate(robotPos.getX(), targetPos.getX());
    double speedY = yController.calculate(robotPos.getY(), targetPos.getY());
    double throttleX;
    double throttleY;
    
    if(Math.abs(speedX) > Math.abs(speedY))
    {
      double ratio = (speedX==0 || speedY==0) ? 0 : speedY/speedX;
      throttleX = Conversions.clamp(xController.calculate(robotPos.getX(), targetPos.getX()));
      throttleY = throttleX*ratio;
    }
    else
    {
      double ratio = (speedX==0 || speedY==0) ? 0 : speedX/speedY;
      throttleY = Conversions.clamp(yController.calculate(robotPos.getY(), targetPos.getY()));
      throttleX = throttleY*ratio;
    }
    final double speedTheta = 
      Conversions.clamp
      (
        thetaController.calculate(pose.getRotation().getRadians(), target.getRotation().getRadians()), 
        -maxAngularVelocity, 
        maxAngularVelocity
      ) * brake;
    final var throttleXY = FieldConstants.GeoFencing.fieldGeoFence.process(new Translation2d(throttleX, throttleY)).times(brake);

    return ChassisSpeeds.fromFieldRelativeSpeeds
    (
      new ChassisSpeeds
      (
        throttleXY.getX() * maxSpeed,
        throttleXY.getY() * maxSpeed,
        speedTheta
      ),
      pose.getRotation()
    );
  }
}
