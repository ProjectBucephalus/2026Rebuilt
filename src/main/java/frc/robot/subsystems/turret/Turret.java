package frc.robot.subsystems.turret;
import frc.robot.constants.IDConstants;

import static frc.robot.constants.Constants.Shooter.*;
import static frc.robot.constants.Constants.Turret.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase
{
  private final TalonFX m_TurretMotor = new TalonFX(IDConstants.turretID);

  private final Supplier<Rotation2d> robotRotationSup;
  private final Supplier<Translation2d> robotPositionSup;

  private final MotionMagicVoltage m_Request = new MotionMagicVoltage(0);

  private double targetAngle = 0;
  private Translation2d targetPoint = Translation2d.kZero;
  private boolean trackingPoint = true;

  public Turret(Supplier<Translation2d> translationSup, Supplier<Rotation2d> rotationSup)
  {
    robotRotationSup = rotationSup;
    robotPositionSup = translationSup;

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

    m_TurretMotor.getConfigurator().apply(turretConfigs);
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

  @Override
  public void periodic()
  {
    if (trackingPoint) {
      // Calculate target angle based on field positions
      double robotTarget = targetPoint.minus(robotPositionSup.get()).getAngle().getDegrees();
      SmartDashboard.putNumber("Target-Robot", robotTarget);

      double fieldTarget = robotTarget - robotRotationSup.get().getDegrees();
      SmartDashboard.putNumber("Target-Turret", fieldTarget);

      double wrappedTarget = TurretCalculator.normaliseAngle(fieldTarget, m_TurretMotor.getPosition().getValueAsDouble() * 360);
      SmartDashboard.putNumber("Angle Wrapped", wrappedTarget);

      targetAngle = wrappedTarget;
    }

    // Set motor to go to target
    m_TurretMotor.setControl(m_Request.withPosition(targetAngle / 360));
  }   
}