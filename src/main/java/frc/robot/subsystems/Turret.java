package frc.robot.subsystems;
import frc.robot.util.TurretCalculator;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;

import static frc.robot.constants.Constants.Shooter.idleSpeed;
import static frc.robot.constants.Constants.Shooter.slot0A;
import static frc.robot.constants.Constants.Shooter.slot0D;
import static frc.robot.constants.Constants.Shooter.slot0I;
import static frc.robot.constants.Constants.Shooter.slot0P;
import static frc.robot.constants.Constants.Shooter.slot0S;
import static frc.robot.constants.Constants.Shooter.slot0V;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase
{
  private TalonFX m_TurretMotor; 
  private double targetAngle = 0;

  final MotionMagicVoltage m_Requests = new MotionMagicVoltage(null);

  public Turret()
  {
      m_TurretMotor = new TalonFX(IDConstants.turretID);

      // in init function
    var TurretConfigs = new TalonFXConfiguration();


        // set slot 0 gains
    TurretConfigs.Slot0.kS = slot0S;
    TurretConfigs.Slot0.kV = slot0V;
    TurretConfigs.Slot0.kA = slot0A;
    TurretConfigs.Slot0.kP = slot0P;
    TurretConfigs.Slot0.kI = slot0I;
    TurretConfigs.Slot0.kD = slot0D;


    TurretConfigs.MotionMagic.MotionMagicAcceleration = Constants.Turret.turretAcceleration;
    TurretConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Turret.turretVelocity;

    m_TurretMotor.getConfigurator().apply(TurretConfigs);
  }

  public void setTargetAngle(double newTargetAngle)
  {
     targetAngle = newTargetAngle; 
  }

  @Override
  public void periodic()
  {
    // Calculate target angle based on field positions

    // Set motor to go to target
    m_TurretMotor.setControl(m_Requests.withPosition(targetAngle));
  }
    
}