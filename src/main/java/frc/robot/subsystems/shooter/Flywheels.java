// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static frc.robot.constants.Constants.Shooter.*;


/** Add your docs here. */
public class Flywheels {
  private final TalonFX m_Main; 
  private final TalonFX m_Aux;

  private final MotionMagicVelocityVoltage m_Request = new MotionMagicVelocityVoltage(0);

  public Flywheels(int mainID, int auxID)
  {
    m_Main = new TalonFX(mainID);
    m_Aux = new TalonFX(auxID);

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
    m_Main.getConfigurator().apply(shooterConfigs);

    m_Aux.setControl(new Follower(mainID, MotorAlignmentValue.Opposed));
  }
   // gives the control of the flywheels to m_Requests.
  public void setSpeed(int speed)
    {m_Main.setControl(m_Request.withVelocity(speed));}
}
