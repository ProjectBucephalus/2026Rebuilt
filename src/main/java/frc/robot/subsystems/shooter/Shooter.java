package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.Interpolation;
import frc.robot.constants.Constants.ShooterConstants.HoodConstants;
import frc.robot.constants.Constants.ShooterConstants.TurretConstants;
import frc.robot.constants.IDConstants.ShooterIDs;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;

import static frc.robot.constants.Constants.ShooterConstants.FlywheelConstants.idleSpeed;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

/**
 * Turreted Shooter master-system, internally creates and manages associated subsystems
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Shooter extends SubsystemBase 
{  
  @Logged
  private final Flywheels flywheels; 
  @Logged
  private final Turret turret;
  @Logged
  private final Hood hood;
  
  private final Transform2d shooterOffset;
  private final Translation2d baseTargetOffset;

  private final String ntId;

  private final Supplier<SwerveDriveState> swerveStateSup;
  private SwerveDriveState swerveState;

  /** Current active target for the shooter */
  @Logged(name = "Target")
  private Target target = new Target(TargetState.Manual);

  /**
   * Creates Turreted Shooter master-system, internally creates and manages associated subsystems
   * @param swerveStateSup Supplier for drivebase state to access pose and motion values
   * @param robotToShooter Offset from robot-centre to turret-centre, including rotation
   * @param flywheelLeaderCAN CAN-ID of primary shooter motor
   * @param flywheelFollowerCAN CAN-ID of secondary shooter motor, set to follow first
   * @param turretCAN CAN-ID of turret azimuth motor
   * @param azimuthIO AIO-ID of azimuth potentiometer
   * @param azimuthOffset Potentiometer reading for centre of rotation
   * @param hoodPWM PWM-ID of hood altitude servo
   * @param hoodIO AIO-ID of hood feedback sensor
   * @param invertedHood Inverts the range and direction of motion of the hood servo
   * @param activeSup Supplier for when the shooter should be ready to shoot
   */
  public Shooter
  (
    Supplier<SwerveDriveState> swerveStateSup,
    Transform2d robotToShooter,
    ShooterIDs idBlock,
    double azimuthOffset,
    boolean invertedHood
  ) 
  {
    this.swerveStateSup = swerveStateSup;
    this.swerveState = swerveStateSup.get();
    this.shooterOffset = robotToShooter;

    baseTargetOffset = new Translation2d(0, Math.copySign(ShooterConstants.targetPointOffset, robotToShooter.getY()));
    ntId = idBlock.ntID();

    flywheels = new Flywheels(idBlock.flywheelLeadCAN(), idBlock.flywheelFollowCAN());
    turret = new Turret(idBlock.azimuthCAN(), idBlock.azimuthAIO(), azimuthOffset, this::getTarget);
    hood = new Hood(idBlock.altitudePWM(), idBlock.altitudeAIO(), invertedHood, target);

    target.azimuth = turret.getAzimuth();
  }

  /**
   * Construct a command that sets the speed for the Flywheels <p>
   * NOTE: The provided value is only evaluated when the command is created
   * 
   * @param speed the desired Flywheel speed, in rotations per second
   * @return the {@link Command}
   */
  public Command setFlySpeedCommand(double speed)
    {return runOnce(() -> target.speed = speed);}

  public Command adjustDistanceCommand(DoubleSupplier shiftSup)
  {
    return run(() -> 
    {
      target.distance += shiftSup.getAsDouble();
      target.speed = Interpolation.flywheelSpeedHub.get(target.distance);
      target.altitude = Interpolation.shooterAltitudeHub.get(target.distance);
    });
  }

  public Command adjustAzimuthCommand(DoubleSupplier shiftSup)
    {return run(() -> target.azimuth += shiftSup.getAsDouble());}

  public void setFlySpeed(double speed)
    {target.speed = speed;}

  /** @return Current robot-relative azimuth of the turret, degrees */
  public double getAzimuth()
    {return turret.getAzimuth() - shooterOffset.getRotation().getDegrees();}

  /** @return Current speed of the flywheels (RPS of the main flywheel) */
  public double getSpeed()
    {return flywheels.getSpeed();}

  /** @return Current Target object for the Shooter system */
  public Target getTarget() {return target;}

  /**
   * Trigger factory for whether we are in a valid state to be shooting. This requires that:
   * <ul>
   * <li> The turret is within {@link TurretConstants#azimuthTolerance azimuthTolerance} of it's target azimuth
   * <li> The hood is within {@link HoodConstants#altTolerance altTolerance} of it's target altitude
   * <li> The flywheels are at target speed, as per {@link Flywheels#atSpeed()}
   * <li> The combined rotational velocity of the turret and the drivebase is less than {@link TurretConstants#maxRPS maxRPS}
   * </ul>
   * 
   * @return A {@link Trigger} encoding the above behaviour
   */
  @Logged
  public boolean shootReady()
  {
    return 
      target.flywheelsActive
      && turret.readyToShoot(swerveState.Speeds)
      && hood.atAltitude()
      && flywheels.atSpeed();
  }

  /**
   * Brings flywheels up to at least idle speed
   * @return {@code true} when flywheels are at speed
   */
  public boolean makeShootSafe()
  {
    if (target.speed < idleSpeed)
      {target.speed = idleSpeed;}

    return flywheels.atSpeed();
  }

  /** Sets the flywheels to rev up to target speed */
  public void revFlywheels() {target.flywheelsActive = true;}
  /** Sets the flywheels to idle speed */
  public void idleFlywheels() {target.flywheelsActive = false;}

  private void telemetrise()
  {
    var shooterPose = swerveState.Pose
      .plus(shooterOffset)
      .plus(new Transform2d(Translation2d.kZero, Rotation2d.fromDegrees(turret.getAzimuth())));
    PBDash.putFieldPath(ntId + " Pose", shooterPose, shooterPose.transformBy(new Transform2d(flywheels.getSpeed() / 60, 0, Rotation2d.kZero)));

    var targetPoint = target.state == TargetState.Hub ? FieldUtils.getAllianceHubCentre() : target.point;
    PBDash.putFieldObject(ntId + "Target", new Pose2d(targetPoint.plus(target.offset), Rotation2d.kZero));
  }

  @Override
  public void periodic()
  {
    swerveState = swerveStateSup.get();
    var shooterPose = swerveState.Pose.plus(shooterOffset);

    // Find distance to current target for calculating leading shots
    double distance = switch (target.state) 
    {
      case Manual -> target.distance;
      case Point -> target.point.minus(shooterPose.getTranslation()).getNorm();
      // aim at our alliance's hub
      case Hub -> FieldUtils.getAllianceHubCentre().minus(shooterPose.getTranslation()).getNorm();
    };

    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerveState.Speeds, swerveState.Pose.getRotation());
    // Calculate target offset to avoid balls from each shooter colliding before reaching target
    // and accounting for robot motion
    target.offset = 
      baseTargetOffset
        .rotateBy(swerveState.Pose.getRotation().unaryMinus())
        .minus
        (
          new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
          .times(distance * ShooterConstants.leadFactor) // distance * leadFactor
        );

    // Find distance to current target for calculating leading shots
    target.distance = switch (target.state) 
    {
      case Manual -> target.distance;
      case Point -> target.point.plus(target.offset).minus(shooterPose.getTranslation()).getNorm();
      // aim at our alliance's hub
      case Hub -> FieldUtils.getAllianceHubCentre().plus(target.offset).minus(shooterPose.getTranslation()).getNorm();
    };

    target.altitude = switch (target.state)
    {
      case Manual -> target.altitude;
      case Point -> Interpolation.shooterAltitudeLow.get(target.distance);
      case Hub -> Interpolation.shooterAltitudeHub.get(target.distance);
    };

    target.speed = switch (target.state)
    {
      case Manual -> target.speed;
      case Point -> Interpolation.flywheelSpeedLow.get(target.distance);
      case Hub -> Interpolation.flywheelSpeedHub.get(target.distance);
    };
    if (target.flywheelsActive)
      flywheels.setSpeed(target.speed);
    else
      flywheels.setSpeed(idleSpeed);

    turret.update(shooterPose, Math.toDegrees(swerveState.Speeds.omegaRadiansPerSecond));
    hood.update();
    

    telemetrise();
  }

  @Override
  public void simulationPeriodic() 
  {
    turret.updateSim();
    flywheels.updateSim();
  }
}
