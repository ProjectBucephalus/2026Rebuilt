package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Constants.Shooter.HoodConstants;
import frc.robot.constants.Constants.Shooter.TurretConstants;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.util.Conversions;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class Shooter extends SubsystemBase 
{  
  // these variables are defined here but used in the corsponding files 
  private final Flywheels flywheels; 
  private final Turret turret;
  private final Hood hood;

  private final Supplier<SwerveDriveState> swerveStateSup;
  private SwerveDriveState swerveState;

  private Target target = new Target(TargetState.Hub);

  /** Creates a new shooter. */
  public Shooter
  (
    Supplier<SwerveDriveState> swerveStateSup,
    Translation2d turretOffset,
    int flywheelLeaderCAN, 
    int flywheelFollowerCAN, 
    int turretCAN, 
    int hoodPWM
  ) 
  {
    this.swerveStateSup = swerveStateSup;

    flywheels = new Flywheels(flywheelLeaderCAN, flywheelFollowerCAN);
    turret = new Turret(turretCAN, turretOffset);
    hood = new Hood(hoodPWM, Transform2d.kZero);
  }

  public void setTarget(Target target)
    {this.target = target;}

  public void setFlywheels(double speed)
    {flywheels.setSpeed(speed);}

  public Trigger shootReadyTrigger()
  {
    return new Trigger
      (() -> {
        return Conversions.nearRotation(turret.getAzimuth(), target.azimuth, TurretConstants.azimuthTolerance)
                && Conversions.nearRotation(hood.getAltitude(), target.altitude, HoodConstants.altTolerance)
                && flywheels.atSpeed()
                && (turret.getRPM() + Math.toDegrees(swerveState.Speeds.omegaRadiansPerSecond)) < TurretConstants.maxRPM;
      });
  }

  @Override
  public void periodic()
  {
    swerveState = swerveStateSup.get();

    turret.update(swerveState.Pose, target);
    hood.update(swerveState.Pose, target);
  }
}
