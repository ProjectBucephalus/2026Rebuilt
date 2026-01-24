package frc.robot.subsystems.shooter;

import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import static frc.robot.constants.Constants.TurretConstants.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;

public class Turret
{
  private final TalonFX m_Turret;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  private final Translation2d robotOffset;

  public Turret(int id, Translation2d robotOffset) 
  {
    this.robotOffset = robotOffset;
    m_Turret = new TalonFX(id);

    m_Turret.getConfigurator().apply(turretConfigs);
  }

  /**
   * @param robotPose Current Pose2d of the robot
   * @param targetPoint Translation2d of the target
   * @return Desired turret angle to aim at target, in degrees
   */
  private double calculateTargetAngle(Pose2d robotPose, Translation2d targetPoint)
  {
    var turretPos = robotPose.getTranslation().plus(robotOffset);
    // Angle from turret centre to target relative to field +X axis
    double fieldTarget = targetPoint.minus(turretPos).getAngle().getDegrees();
    // Robot-Relative angle from turret to target
    double robotTarget = fieldTarget - robotPose.getRotation().getDegrees();

    return Conversions.normaliseAngle(robotTarget, getRotation().getDegrees(), maxTurretAzimuth);
  }

  public void unwind()
  {
    double angle = getRotation().getDegrees();
    m_Turret.setControl(request.withPosition(Conversions.normaliseAngle(angle + 360, angle, maxTurretAzimuth) / 360)); 
  }

  public void home()
    {m_Turret.setControl(request.withPosition(0));}

  public Rotation2d getRotation() 
    {return new Rotation2d(m_Turret.getPosition().getValue());}
  
  //using the states defined in Target.java to set the place that the turret is tracking
  public void update(Pose2d robotPose, Target target)
  {
    double targetAzimuth = switch (target.state) 
    {
      case Manual -> target.azimuth;
      case Point -> calculateTargetAngle(robotPose, target.point);
      case Hub -> calculateTargetAngle(robotPose, FieldUtils.getAllianceHubCentre());
    };

    target.azimuth = targetAzimuth;

    // gives control of the motors to request to be called in Robot.java
    m_Turret.setControl(request.withPosition(targetAzimuth / 360));     
  }   
}