// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.generic;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class BiMotor extends VelocityMotor 
{
  protected final TalonFX m_Follower;

  /**
   * Creates a wrapper around two TalonFXs to provide synchronised velocity control
   * 
   * @param leaderCAN CAN ID of the leader motor
   * @param followerCAN CAN ID of the follower motor
   * @param config The config to apply to the wrapped motor
   * @param flipped Whether the follower motor should run in the opposite direction as the leader
   */
  public BiMotor(int leaderCAN, int followerCAN, TalonFXConfiguration config, boolean flipped) 
  {
    super(leaderCAN, config);
    m_Follower = new TalonFX(followerCAN);

    m_Follower.getConfigurator().apply(config);

    m_Follower.setControl(new Follower(leaderCAN, flipped ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
  }

  @Override
  public boolean devicesValid()
  {
    return super.devicesValid() && m_Follower.isConnected();
  }
}
