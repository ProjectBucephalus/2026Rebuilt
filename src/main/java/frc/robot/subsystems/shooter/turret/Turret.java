package frc.robot.subsystems.shooter.turret;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.shooter.Shooter.Target;
import frc.robot.util.FieldUtils;

import static frc.robot.constants.Constants.TurretConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret
{
  private final TalonFX m_Turret;

  private final MotionMagicVoltage m_Request = new MotionMagicVoltage(0);

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
  // calculates the Angle to the target 
  private double calculateTargetAngle(Pose2d robotPose, Translation2d targetPoint)
  {
    double robotTarget = targetPoint.minus(robotPose.getTranslation()).getAngle().getDegrees();
    double fieldTarget = robotTarget - robotPose.getRotation().getDegrees();
    double wrappedTarget = TurretCalculator.normaliseAngle(fieldTarget, m_Turret.getPosition().getValueAsDouble() * 360);
    return wrappedTarget;
  }

  public void update(Pose2d robotPose, Target target)
  {
    double targetAngle = switch (target) {
      case Manual -> target.heading;
      case Point -> calculateTargetAngle(robotPose, target.point);
      case Hub -> calculateTargetAngle(robotPose, FieldUtils.getAllianceHubCentre());
    };
    // gives control of the motors to m_Requests
    m_Turret.setControl(m_Request.withPosition(targetAngle / 360));     
  }   
}