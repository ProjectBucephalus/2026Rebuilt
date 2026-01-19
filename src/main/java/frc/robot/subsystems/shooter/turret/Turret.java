package frc.robot.subsystems.shooter.turret;
import frc.robot.constants.IDConstants;

import static frc.robot.constants.Constants.TurretConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret
{
  private final TalonFX m_Turret;

  
  private final MotionMagicVoltage m_Request = new MotionMagicVoltage(0);

  private double targetAngle = 0;
  private Translation2d targetPoint = Translation2d.kZero;
  private boolean trackingPoint = true;

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
 
  public void setTargetAngle(double newTargetAngle)
  {
    targetAngle = newTargetAngle; 
    trackingPoint = false;
  }
  
  public void setTargetPoint(Translation2d newTargetPoint)
  {
    targetPoint = newTargetPoint;
    trackingPoint = true;
  }

  
  public void update(Translation2d robotPosition, Rotation2d robotRotation)
  {
    if (trackingPoint) {
      // Calculate target angle based on field positions
      double robotTarget = targetPoint.minus(robotPosition).getAngle().getDegrees();
      SmartDashboard.putNumber("Target-Robot", robotTarget);

      double fieldTarget = robotTarget - robotRotation.getDegrees();
      SmartDashboard.putNumber("Target-Turret", fieldTarget);

      double wrappedTarget = TurretCalculator.normaliseAngle(fieldTarget, m_Turret.getPosition().getValueAsDouble() * 360);
      SmartDashboard.putNumber("Angle Wrapped", wrappedTarget);

      targetAngle = wrappedTarget;
    }

    // Set motor to go to target
    m_Turret.setControl(m_Request.withPosition(targetAngle / 360));
  }   
}