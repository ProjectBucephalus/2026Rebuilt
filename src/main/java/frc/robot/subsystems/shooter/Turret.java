package frc.robot.subsystems.shooter;

import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import static frc.robot.constants.Constants.TurretConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Turret
{
  private final TalonFX m_Turret;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  public Turret(int id) 
  {
    m_Turret = new TalonFX(id);

    // in init function
    var turretConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    turretConfigs.Slot0.kS = slot0S;
    turretConfigs.Slot0.kV = slot0V;
    turretConfigs.Slot0.kA = slot0A;
    turretConfigs.Slot0.kP = slot0P;
    turretConfigs.Slot0.kI = slot0I;
    turretConfigs.Slot0.kD = slot0D;

    turretConfigs.MotionMagic.MotionMagicAcceleration = turretAcceleration;
    turretConfigs.MotionMagic.MotionMagicCruiseVelocity = turretVelocity;

    m_Turret.getConfigurator().apply(turretConfigs);
  }

  
  // calculates the Angle to the target. Using targetPoint from Target.java
  private double calculateTargetAngle(Pose2d robotPose, Translation2d targetPoint)
  {
    double robotTarget = targetPoint.minus(robotPose.getTranslation()).getAngle().getDegrees();
    double fieldTarget = robotTarget - robotPose.getRotation().getDegrees();
    double wrappedTarget = Conversions.normaliseAngle(fieldTarget, m_Turret.getPosition().getValueAsDouble() * 360, maxTurretAzimuth);
    return wrappedTarget;
  }
  
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