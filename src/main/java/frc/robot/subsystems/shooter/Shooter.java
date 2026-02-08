package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ShooterConstants.HoodConstants;
import frc.robot.constants.Constants.ShooterConstants.TurretConstants;
import frc.robot.constants.IDConstants.ShooterIDs;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

/**
 * Turreted Shooter master-system, internally creates and manages associated subsystems
 * @author 5985
 */
public class Shooter extends SubsystemBase 
{  
  private final Flywheels flywheels; 
  private final Turret turret;
  private final Hood hood;
  
  private final Transform2d shooterOffset;
  private final Translation2d baseTargetOffset;

  private final Supplier<SwerveDriveState> swerveStateSup;
  private SwerveDriveState swerveState;

  /** Current active target for the shooter */
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
    this.shooterOffset = robotToShooter;

    baseTargetOffset = new Translation2d(0, Math.copySign(ShooterConstants.targetPointOffset, robotToShooter.getY()));

    flywheels = new Flywheels(idBlock.flywheelLeadCAN(), idBlock.flywheelFollowCAN());
    turret = new Turret(idBlock.azimuthCAN(), idBlock.azimuthAIO(), azimuthOffset, this::getTarget);
    hood = new Hood(idBlock.altitudePWM(), idBlock.altitudeAIO(), invertedHood, this::getTarget);
  }

  /**
   * Construct a command that sets the target for the Hood and Turret to track <p>
   * NOTE: The provided target is only evaluated when the command is created
   * 
   * @param target the {@link Target} to be set
   * @return the {@link Command}
   */
  public Command setTargetCommand(Target target)
    {return runOnce(() -> this.target = target);}

  /**
   * Sets the manual position of the turret
   * @param azimuth Turret azimuth, degrees
   * @param altitude Hood altitude, degrees
   */
  public void setManual(double azimuth, double altitude)
  {
    target.azimuth = azimuth;
    target.altitude = altitude;
  }

  /**
   * Construct a command that sets the speed for the Flywheels <p>
   * NOTE: The provided value is only evaluated when the command is created
   * 
   * @param speed the desired Flywheel speed, in rotations per second
   * @return the {@link Command}
   */
  public Command setFlySpeedCommand(double speed)
    {return runOnce(() -> flywheels.setSpeed(speed));}

  public void setFlySpeed(double speed)
  {flywheels.setSpeed(speed);}
  public void setFlyVoltage(double speed)
  {flywheels.setVoltage(speed);}

  /** @return Current robot-relative azimuth of the turret, degrees */
  public Rotation2d getAzimuth()
    {return new Rotation2d(turret.getAzimuth() - shooterOffset.getRotation().getDegrees());}

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
  public Trigger shootReadyTrigger()
  {
    return new Trigger
      (() -> {
        return turret.atAzimuth()
                && hood.atAltitude()
                && flywheels.atSpeed()
                && turret.safeToShoot(swerveState.Speeds);
      });
  }

  @Override
  public void periodic()
  {
    swerveState = swerveStateSup.get();
    var shooterPose = swerveState.Pose.plus(shooterOffset);

    // TODO: Test the extent to which leading shots is needed, and remove distance calculation from here or Hood as appropriate
    // Find distance to current target for calculating leading shots
    double distance = switch (target.state) 
    {
      case Manual -> 0;
      case Point -> target.point.minus(shooterPose.getTranslation()).getNorm();
      // aim at our alliance's hub
      case Hub -> FieldUtils.getAllianceHubCentre().minus(shooterPose.getTranslation()).getNorm();
    };

    // Calculate target offset to avoid balls from each shooter colliding before reaching target
    // and accounting for robot motion
    target.offset = 
      baseTargetOffset
        .rotateBy(swerveState.Pose.getRotation().unaryMinus())
        .plus
        (
          new Translation2d(swerveState.Speeds.vxMetersPerSecond, swerveState.Speeds.vyMetersPerSecond)
          .times(distance * ShooterConstants.leadFactor) // distance * leadFactor
        );

    turret.update(shooterPose, Math.toDegrees(swerveState.Speeds.omegaRadiansPerSecond));
    hood.update(shooterPose);
    flywheels.update();
  }
}
