package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Target.TargetState;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase 
{  
  // these variables are defined here but used in the corsponding files 
  private final Flywheels flywheels; 
  private final Turret turret;
  private final Hood hood;

  private final Supplier<Pose2d> robotPoseSup;

  private Target target = new Target(TargetState.Hub);

  /** Creates a new shooter. */
  public Shooter
  (
    Supplier<Pose2d> robotPoseSup,
    Translation2d turretOffset,
    int flywheelLeaderCAN, 
    int flywheelFollowerCAN, 
    int turretCAN, 
    int hoodPWM
  ) 
  {
    this.robotPoseSup = robotPoseSup;

    flywheels = new Flywheels(flywheelLeaderCAN, flywheelFollowerCAN);
    turret = new Turret(turretCAN, turretOffset);
    hood = new Hood(hoodPWM, Transform2d.kZero);
  }

  public void setTarget(Target target)
    {this.target = target;}

  public void setFlywheels(double speed)
    {flywheels.setSpeed(speed);}

  @Override
  public void periodic()
  {
    turret.update(robotPoseSup.get(), target);
    hood.update(robotPoseSup.get(), target);
  }
}
