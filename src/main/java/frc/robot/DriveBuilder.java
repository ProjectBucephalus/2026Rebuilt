package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.constants.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Conversions;

public class DriveBuilder
{
  private final static SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest
    .FieldCentric() 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final static SwerveRequest.FieldCentricFacingAngle facingAngleRequest = new SwerveRequest
    .FieldCentricFacingAngle()
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withHeadingPID(SwerveConstants.rotationKP, SwerveConstants.rotationKI, SwerveConstants.rotationKD);

  private final static PIDController thetaController = new PIDController(0.02, SwerveConstants.rotationKI, SwerveConstants.rotationKD);

  private static CommandSwerveDrivetrain s_Swerve;
  private static Supplier<Translation2d> joystickSup;
  private static DoubleSupplier rotationSup;
  private static DoubleSupplier brakeSup;
  private static Supplier<Pose2d> robotPoseSup;

  /**
   * Sets up the persistent internal values. Must be called before any of the other functions are used
   * 
   * @param s_Swerve The swerve subsystem
   * @param joystickSup Supplier for the robot translation input, [-1..1][-1..1]. Follows field relative coordinate standard
   * @param rotationSup Supplier for the robot rotation input, [-1..1]
   * @param brakeSup Supplier for the braking input, [0..1]. 0 is no braking, 1 is full braking
   * @param robotPoseSup Supplier for the robot's pose at any given point in time
   */
  public static void init
  (
    CommandSwerveDrivetrain s_Swerve,
    Supplier<Translation2d> joystickSup,
    DoubleSupplier rotationSup,
    DoubleSupplier brakeSup,
    Supplier<Pose2d> robotPoseSup
  )
  {
    DriveBuilder.s_Swerve = s_Swerve;
    DriveBuilder.joystickSup = joystickSup;
    DriveBuilder.rotationSup = rotationSup;
    DriveBuilder.brakeSup = brakeSup;
    DriveBuilder.robotPoseSup = robotPoseSup;
  }

  /** Creates a basic Manual drive command */
  public static Command manual()
  {
    return s_Swerve.run(() -> {
      /* Get and process Rotation input */
      double rotationVal = rotationSup.getAsDouble();

      if (Math.abs(rotationVal) <= ControlConstants.stickDeadband) 
        {rotationVal = 0;}
      else
        // interpolating FROM max TO min implicitly "flips" the brake value
        {rotationVal *= MathUtil.interpolate(ControlConstants.maxRotThrottle, ControlConstants.minRotThrottle, brakeSup.getAsDouble());}

      var motionXY = joystickSup.get();

      s_Swerve.setControl
      (
        fieldCentricRequest
          .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
          .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
          .withRotationalRate(rotationVal * SwerveConstants.maxAngularVelocity)
      );
    });
  }

  /** 
   * Creates a Manual drive with the robot's centre of rotation offset 
   * @param centreOffset The amount to offset the robot's centre by, in meters
   */
  public static Command offset(Translation2d centreOffset)
  {
    return 
      runOnce(() -> fieldCentricRequest.withCenterOfRotation(centreOffset))
      .andThen(manual())
      .finallyDo(b -> fieldCentricRequest.withCenterOfRotation(Translation2d.kZero));
  }

  /**
   * Creates a Heading-locked drive command that maintains a (potentially dynamic) desired heading
   * @param targetHeadingSup A supplier for the target heading, in degrees
   */
  public static Command headingLocked(Supplier<Rotation2d> targetHeadingSup)
  {
    return s_Swerve.run(() -> {
      var motionXY = joystickSup.get();

      s_Swerve.setControl
      (
        facingAngleRequest
          .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
          .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
          .withTargetDirection(targetHeadingSup.get())
      );
    });
  }

  /**
   * Creates a Heading-nudged drive command that maintains a (potentially dynamic) desired heading, except while the rotation stick is being used
   * @param targetHeadingSup A function that takes the robot's current heading, and returns the target heading, both in degrees
   */
  public static Command headingNudged(Function<Double, Double> targetHeadingSup) 
  {
    return s_Swerve.run(() -> {
      double rotationVal = rotationSup.getAsDouble();
      double robotRotation = robotPoseSup.get().getRotation().getDegrees();

      // Rotation stick not being actively controlled
      if (Math.abs(rotationVal) <= ControlConstants.stickDeadband) 
        rotationVal = thetaController.calculate(robotRotation, targetHeadingSup.apply(robotRotation));
      else
        rotationVal *= MathUtil.interpolate(ControlConstants.maxRotThrottle, ControlConstants.minRotThrottle, brakeSup.getAsDouble());

      var motionXY = joystickSup.get();
      s_Swerve.setControl
      (
        fieldCentricRequest
          .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
          .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
          .withRotationalRate(rotationVal * SwerveConstants.maxAngularVelocity)
      );
    });
  }

  /**
   * Creates a drive command that nudges the robot's heading to at least a given amount away from the cardinal directions (0, 90, 180, or 270)
   * @param tolerance How far the robot's heading must be from cardinal, degrees
   * @see DriveBuilder#headingNudged
   */
  public static Command nonCardinal(double tolerance)
  {
    return headingNudged(robotRotation -> {
      // Wrap the robot's rotation to [0..90) (effectively, clockwise degrees past previous cardinal) 
      double wrappedRotation = Conversions.mod(robotRotation, 90);

      // If we're less than tolerance past the previous cardinal, rotate to be tolerance past it
      if (wrappedRotation < tolerance)
      {
        double error = tolerance - wrappedRotation;
        return robotRotation + error;
      }
      // If we're less than tolerance before the next cardinal, rotate to be tolerance before it
      else if (wrappedRotation > 90 - tolerance)
      {
        double error = wrappedRotation - (90 - tolerance);
        return robotRotation - error;
      }
      // If not close to cardinal, don't change rotation
      else
        return robotRotation;
    });
  }

  /** 
   * Creates a drive command that nudges the robot's heading to the closest of 180 and -180 degrees
   * @see DriveBuilder#headingNudged
   */
  public static Command trenchNudge()
  {
    return headingNudged(robotRotation -> {
      // Wrap the robot's rotation to [0..180) (effectively, clockwise degrees past previous straight) 
      double wrappedRotation = Conversions.mod(robotRotation, 180);

      if (wrappedRotation > 90)
        // If we're more than halfway to the next straight, rotate to it
        return robotRotation + (180 - wrappedRotation);
      else
        // Less than halfway to next straight, rotate to previous straight
        return robotRotation - wrappedRotation;       
    });
  }

  /**
   * Creates a PathFollow drive command to navigate to the given target pose
   * @param target Pose2d for the command to navigate to
   */
  public static Command pathFollow(Pose2d target)
    {return pathFollowInner(Arrays.asList(target), Arrays.asList(0.0), () -> 0.0);}

  /**
   * Creates a PathFollow drive command to follow the given path
   * @param path Predefined path for command to follow
   */
  public static Command pathFollow(Path path)
    {return pathFollow(path, () -> 0.0);}

  /**
   * Creates a PathFollow drive command to follow the given path with braking
   * @param path     Predefined path for command to follow
   * @param brakeSup Supplier for the speed reduction to apply, [0..1].  0 is no braking, 1 is full braking
   */
  public static Command pathFollow(Path path, DoubleSupplier brakeSup)
  {
    final ArrayList<Pose2d> waypoints = new ArrayList<>(path.sequence().length * 3 - 2);
    final ArrayList<Double> radiusPerSegment = new ArrayList<>(path.sequence().length);

    for (int i = 0; i < path.sequence().length - 1; i++) 
    {
      final var current = path.sequence()[i];
      final var next = path.sequence()[i + 1];

      // Find the distance between the current point and the next
      // If the input radius is greater than 1/3 the distance between points, use 1/3 for next step
      final double segmentLength = current.getDistance(next);
      final double waypointDist = Conversions.clamp(path.pointRadius(), ControlConstants.lineupTolerance, segmentLength / 3);
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

    return pathFollowInner(waypoints, radiusPerSegment, brakeSup);
  }

  /**
   * Internal helper producing the actual path following command, allowing for multiple different external wrapper functions that create the lists used 
   * @param waypoints A list of all the waypoints the command should follow
   * @param radiusPerSegment A list of the lineup tolerances for the waypoints of each segment of the path (each segment is 3 waypoints, except the final one which is a single waypoint)
   * @param brakeSup Supplier for the speed reduction to apply, [0..1].  0 is no braking, 1 is full braking
   */
  private static Command pathFollowInner(List<Pose2d> waypoints, List<Double> radiusPerSegment, DoubleSupplier brakeSup)
  {
    return new Command() 
    {
      private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();    

      private boolean onPath = false;
      private int currentWaypoint = 0;
      
      @Override
      public InterruptionBehavior getInterruptionBehavior() 
        {return InterruptionBehavior.kCancelIncoming;}

      @Override
      public void initialize() 
      {
        currentWaypoint = 0;
        onPath = false;
      }

      @Override
      public void execute() 
      {
        var robotPose = robotPoseSup.get();

        // If the robot is close to the path, follow one point ahead to give smoother cornering
        final var targetIndex = Math.min(onPath ? currentWaypoint + 1 : currentWaypoint, waypoints.size() - 1);
        final var targetPose = waypoints.get(targetIndex);
        
        s_Swerve.setControl(driveRequest.withSpeeds(s_Swerve.calculateDrivePID(targetPose, robotPose, brakeSup.getAsDouble())));
            
        // Switch to next waypoint when within the given distance of the current one
        final var currentSegment = Math.floorDiv(currentWaypoint, 3);

        if (Conversions.nearTranslation(robotPose.getTranslation(), targetPose.getTranslation(), radiusPerSegment.get(currentSegment))) 
        {
          currentWaypoint = Math.min(currentWaypoint + 1, waypoints.size());
          onPath = true;
        }
      }

      @Override
      public boolean isFinished() 
      {
        // Finish when robot is at the final waypoint
        return Conversions.atPose(robotPoseSup.get(), waypoints.get(waypoints.size()-1));
      }
    };
  }
}
