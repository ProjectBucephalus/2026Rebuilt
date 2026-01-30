package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.Shooter.HoodConstants;
import frc.robot.constants.Constants.Shooter.TurretConstants;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.util.Conversions;

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

  private final Supplier<SwerveDriveState> swerveStateSup;
  private SwerveDriveState swerveState;

  /** Current active target for the shooter */
  private Target target = new Target(TargetState.Hub);

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
   */
  public Shooter
  (
    Supplier<SwerveDriveState> swerveStateSup,
    Transform2d robotToShooter,
    int flywheelLeaderCAN, 
    int flywheelFollowerCAN, 
    int turretCAN,
    int azimuthIO,
    double azimuthOffset,
    int hoodPWM
  ) 
  {
    this.swerveStateSup = swerveStateSup;
    this.shooterOffset = robotToShooter;

    flywheels = new Flywheels(flywheelLeaderCAN, flywheelFollowerCAN);
    turret = new Turret(turretCAN, azimuthIO, azimuthOffset);
    hood = new Hood(hoodPWM);
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
   * Construct a command that sets the speed for the Flywheels <p>
   * NOTE: The provided value is only evaluated when the command is created
   * 
   * @param speed the desired Flywheel speed, in rotations per second
   * @return the {@link Command}
   */
  public Command setFlySpeedCommand(double speed)
    {return runOnce(() -> flywheels.setSpeed(speed));}

  /**
   * Trigger factory for whether we are in a valid state to be shooting. This requires that:
   * <ul>
   * <li> The turret is within {@link TurretConstants#azimuthTolerance azimuthTolerance} of it's target azimuth
   * <li> The hood is within {@link HoodConstants#altTolerance altTolerance} of it's target altitude
   * <li> The flywheels are at target speed, as per {@link Flywheels#atSpeed()}
   * <li> The combined rotational velocity of the turret and the drivebase is less than {@link TurretConstants#maxRPM maxRPM}
   * </ul>
   * 
   * @return A {@link Trigger} encoding the above behaviour
   */
  public Trigger shootReadyTrigger()
  {
    return new Trigger
      (() -> {
        return Conversions.nearRotation(turret.getAzimuth(), target.azimuth, TurretConstants.azimuthTolerance)
                && hood.atAltitude()
                && flywheels.atSpeed()
                && (turret.getSpeed() + Math.toDegrees(swerveState.Speeds.omegaRadiansPerSecond)) < TurretConstants.maxRPM;
      });
  }

  @Override
  public void periodic()
  {
    swerveState = swerveStateSup.get();
    var shooterPose = swerveState.Pose.plus(shooterOffset);

    turret.update(shooterPose, target, Math.toDegrees(swerveState.Speeds.omegaRadiansPerSecond));
    hood.update(shooterPose, target);
  }
}
