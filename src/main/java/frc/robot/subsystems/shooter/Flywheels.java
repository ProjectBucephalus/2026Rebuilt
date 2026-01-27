// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

import static frc.robot.constants.Constants.Shooter.*;

/** Add your docs here. */
public class Flywheels 
{
  private final TalonFX m_Leader; 
  private final TalonFX m_Follower;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

  public Flywheels(int leaderCAN, int followerCAN)
  {
    m_Leader = new TalonFX(leaderCAN);
    m_Follower = new TalonFX(followerCAN);

    var shooterConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    shooterConfigs.Slot0.kS = flywheelKS; 
    shooterConfigs.Slot0.kV = flywheelKV; 
    shooterConfigs.Slot0.kA = flywheelKA; 
    shooterConfigs.Slot0.kP = flywheelKP;
    shooterConfigs.Slot0.kI = flywheelKI; 
    shooterConfigs.Slot0.kD = flywheelKD; 

    // set Motion Magic settings
    shooterConfigs.MotionMagic.MotionMagicAcceleration = flywheelAcceleration;
    shooterConfigs.MotionMagic.MotionMagicJerk = flywheelJerk; 
    // sets m_Aux to a follower of m_Main
    m_Leader.getConfigurator().apply(shooterConfigs);

    m_Follower.setControl(new Follower(leaderCAN, MotorAlignmentValue.Opposed));
  }
   // gives the control of the flywheels to request.
  public void setSpeed(double speed)
    {m_Leader.setControl(request.withVelocity(speed));}

  public boolean atSpeed() 
  {
    double targetSpeed = request.Velocity;
    var currentSpeed = m_Leader.getVelocity().getValueAsDouble();
    return MathUtil.isNear(targetSpeed, currentSpeed,flySpeedTolerance);
    
    
  }
}
