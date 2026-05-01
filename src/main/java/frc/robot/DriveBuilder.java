package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot.NavState;
import frc.robot.Robot.RobotState;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.constants.Path;
import frc.robot.constants.Path.Node;
import frc.robot.controlTransmutation.geoFence.GeoFence;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

public class DriveBuilder
{
  private static final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest
    .FieldCentric() 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private static final SwerveRequest.FieldCentricFacingAngle facingAngleRequest = new SwerveRequest
    .FieldCentricFacingAngle()
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withHeadingPID(SwerveConstants.rotationKP, SwerveConstants.rotationKI, SwerveConstants.rotationKD);

  private static final PIDController thetaController = new PIDController(SwerveConstants.rotationKP, SwerveConstants.rotationKI, SwerveConstants.rotationKD);

  private static CommandSwerveDrivetrain s_Swerve;
  private static Supplier<Translation2d> joystickSup;
  private static DoubleSupplier rotationSup;
  private static DoubleSupplier brakeSup;
  private static Supplier<Pose2d> robotPoseSup;

  private static RobotState robotState;

  /**
   * Sets up the persistent internal values. Must be called before any of the other functions are used
   * 
   * @param s_Swerve The swerve subsystem
   * @param joystickSup Supplier for the robot translation input, [-1..1][-1..1]. Follows field relative coordinate standard
   * @param rotationSup Supplier for the robot rotation input, [-1..1]
   * @param brakeSup Supplier for the braking input, [0..1]. 0 is no braking, 1 is full braking
   * @param robotPoseSup Supplier for the robot's pose at any given point in time
   * @param robotState State object for the robot to allow navState to be updated
   */
  public static void init
  (
    CommandSwerveDrivetrain s_Swerve,
    Supplier<Translation2d> joystickSup,
    DoubleSupplier rotationSup,
    DoubleSupplier brakeSup,
    Supplier<Pose2d> robotPoseSup,
    RobotState robotState
  )
  {
    DriveBuilder.s_Swerve = s_Swerve;
    DriveBuilder.joystickSup = joystickSup;
    DriveBuilder.rotationSup = rotationSup;
    DriveBuilder.brakeSup = brakeSup;
    DriveBuilder.robotPoseSup = robotPoseSup;
    DriveBuilder.robotState = robotState;
  }

  private static void manualStateUpdate(boolean hasInput)
  {
    if (GeoFence.isBlocked()) robotState.nav = NavState.Blocked;
    else if (hasInput || robotState.nav != NavState.AtTarget)
    if (FieldUtils.hubActive(Alliance.Blue)) 
      if (FieldUtils.hubActive(Alliance.Red)) robotState.nav = NavState.DualShift;
      else robotState.nav = NavState.BlueShift;
    else if (FieldUtils.hubActive(Alliance.Red)) robotState.nav = NavState.RedShift;
    else robotState.nav = NavState.Manual;
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
      robotState.joystickOutput = motionXY;

      s_Swerve.setControl
      (
        fieldCentricRequest
          .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
          .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
          .withRotationalRate(rotationVal * SwerveConstants.maxAngularVelocity)
      );

      manualStateUpdate(!motionXY.equals(Translation2d.kZero));
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
      robotState.joystickOutput = motionXY;

      s_Swerve.setControl
      (
        facingAngleRequest
          .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
          .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
          .withTargetDirection(targetHeadingSup.get())
      );

      robotState.nav = GeoFence.isBlocked() ? NavState.Blocked : NavState.HeadingLocked;
    });
  }

  /**
   * Creates a Heading-nudged drive command that maintains a (potentially dynamic) desired heading, except while the rotation stick is being used
   * @param targetHeadingSup A function that takes the robot's current heading, and returns the target heading, both in degrees
   */
  public static Command headingNudged(DoubleUnaryOperator targetHeadingSup) 
  {
    return s_Swerve.run(() -> {
      double rotationVal = rotationSup.getAsDouble();
      double robotRotation = robotPoseSup.get().getRotation().getDegrees();

      // Rotation stick not being actively controlled
      if (Math.abs(rotationVal) <= ControlConstants.stickDeadband) 
        rotationVal = Math.toRadians(thetaController.calculate(robotRotation, targetHeadingSup.applyAsDouble(robotRotation)));
      else
        rotationVal *= MathUtil.interpolate(ControlConstants.maxRotThrottle, ControlConstants.minRotThrottle, brakeSup.getAsDouble());

      var motionXY = joystickSup.get();
      robotState.joystickOutput = motionXY;
      s_Swerve.setControl
      (
        fieldCentricRequest
          .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
          .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
          .withRotationalRate(rotationVal * SwerveConstants.maxAngularVelocity)
      );

      robotState.nav = GeoFence.isBlocked() ? NavState.Blocked : NavState.Nudged;
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
   * @param coastOut [Optional] If true will not stop at final waypoint, intended for path chaining
   */
  public static Command pathFollow(Pose2d target)
    {return pathFollow(target, false);}
  
  /**
   * Creates a PathFollow drive command to navigate to the given target pose
   * @param target Pose2d for the command to navigate to
   * @param coastOut [Optional] If true will not stop at final waypoint, intended for path chaining
   */
  public static Command pathFollow(Pose2d target, boolean coastOut)
    {return pathFollow(target, () -> 1.0, coastOut);}

  /**
   * Creates a PathFollow drive command to navigate to the given target pose with braking
   * @param target Pose2d for the command to navigate to
   * @param throttleSup Supplier for the throttle to apply, [0..1]. 1 is full speed, 0 is stopped
   * @param coastOut [Optional] If true will not stop at final waypoint, intended for path chaining
   */
  public static Command pathFollow(Pose2d target, DoubleSupplier throttleSup)
    {return pathFollow(target, throttleSup, false);}
  
  /**
   * Creates a PathFollow drive command to navigate to the given target pose with braking
   * @param target Pose2d for the command to navigate to
   * @param throttleSup Supplier for the throttle to apply, [0..1]. 1 is full speed, 0 is stopped
   * @param coastOut [Optional] If true will not stop at final waypoint, intended for path chaining
   */
  public static Command pathFollow(Pose2d target, DoubleSupplier throttleSup, boolean coastOut)
    {return pathFollowInner(Arrays.asList(new Node(target, 0.3)), throttleSup, coastOut);}

  /**
   * Creates a PathFollow drive command to follow the given path with braking
   * @param path        Predefined path for command to follow
   * @param coastOut [Optional] If true will not stop at final waypoint, intended for path chaining
   */
  public static Command pathFollow(Path path)
    {return pathFollow(path, false);}

  /**
   * Creates a PathFollow drive command to follow the given path with braking
   * @param path        Predefined path for command to follow
   * @param coastOut [Optional] If true will not stop at final waypoint, intended for path chaining
   */
  public static Command pathFollow(Path path, boolean coastOut)
  {
    final ArrayList<Node> waypoints = new ArrayList<>(path.nodes().length * 3 - 2);

    for (int i = 0; i < path.nodes().length - 1; i++) 
    {
      final var current = path.nodes()[i];
      final var next = path.nodes()[i + 1];

      // Find the distance between the current point and the next
      final double segmentLength = current.pose().getTranslation().getDistance(next.pose().getTranslation());
      // Clamp the input radius between our minimum tolerance and 1/3rd of the length of this segment
      final double clampedRadius = Conversions.clamp(current.radius(), ControlConstants.lineupTolerance, segmentLength / 3);
      
      // Calculate how far along the segment to place the midpoints, as a ratio of the clamped radius to the segment length
      final double lengthRatio = clampedRadius / segmentLength;

      // Add current waypoint and additional projected midpoints (for smoothing) to path list
      waypoints.addAll
      (
        List.of
        (
          new Node(current.pose(), clampedRadius), 
          new Node(current.pose().interpolate(next.pose(), lengthRatio), clampedRadius),
          new Node(next.pose().interpolate(current.pose(), lengthRatio), clampedRadius)
        )
      );
    }

    // Final waypoint uses the same radius as the segment leading up to it
    waypoints.add(new Node(path.nodes()[path.nodes().length - 1].pose(), waypoints.get(waypoints.size() - 2).radius()));

    return pathFollowInner(waypoints, path::throttle, coastOut);
  }

  /**
   * Internal helper producing the actual path following command, allowing for multiple different external wrapper functions that create the lists used 
   * @param waypoints A list of all the waypoints the command should follow
   * @param radiusPerSegment A list of the lineup tolerances for the waypoints of each segment of the path (each segment is 3 waypoints, except the final one which is a single waypoint)
   * @param throttleSup Supplier for the throttle to apply, [0..1]. 1 is full speed, 0 is stopped
   * @param coastOut If true will not stop at final waypoint, intended for path chaining
   */
  private static Command pathFollowInner(List<Node> waypoints, DoubleSupplier throttleSup, boolean coastOut)
  {
    return new Command() 
    {
      private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();    

      private boolean onPath = false;
      private int currentWaypoint = 0;

      private boolean coast = coastOut;
      
      {addRequirements(s_Swerve);}

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
        var targetIndex = Math.min(onPath ? currentWaypoint + 1 : currentWaypoint, waypoints.size() - 1);
        var targetNode = waypoints.get(targetIndex);
        
        double throttle = Conversions.clamp(throttleSup.getAsDouble(), 0.0, ControlConstants.maxThrottle);
        double rotThrottle = throttle + (ControlConstants.maxRotThrottle - throttle) / 2;
        s_Swerve.setControl(driveRequest.withSpeeds(s_Swerve.calculateDrivePID(targetNode.pose(), robotPose, throttle, rotThrottle)));

        if (Conversions.nearTranslation(robotPose.getTranslation(), targetNode.pose().getTranslation(), targetNode.radius() * (GeoFence.isBlocked() ? 2 : 1))) 
        {
          currentWaypoint = Math.min(currentWaypoint + 1, waypoints.size());
          onPath = true;
        }

        robotState.nav = GeoFence.isBlocked() ? NavState.Blocked : NavState.Following;
      }

      @Override
      public boolean isFinished() 
      {
        var endNode = waypoints.get(waypoints.size()-1);
        // Finish when robot is at the final waypoint, or near the final waypoint if there is a followup path
        return coast
            ? Conversions.nearPose(robotPoseSup.get(), endNode.pose(), endNode.radius() * (GeoFence.isBlocked() ? 2 : 1), ControlConstants.angleLineupTolerance * 5)
            : Conversions.atPose(robotPoseSup.get(), endNode.pose());
      }

      @Override
      public void end(boolean interrupted) 
        {if (!interrupted) robotState.nav = NavState.AtTarget;}
    };
  }

  /** @return Command that makes the drivebase stop wait until interupted */
  public static Command waitCommand()
  {
    return new Command() 
    {
      @Override
      public void initialize()
      {
        s_Swerve.setControl
        (
          fieldCentricRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
      }
    }
    .andThen(s_Swerve.run(() -> Commands.waitUntil(() -> false)));
  }
}
