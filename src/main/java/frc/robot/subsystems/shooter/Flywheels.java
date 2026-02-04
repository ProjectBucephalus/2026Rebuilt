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

import static frc.robot.constants.Constants.Shooter.FlywheelConstants.*;
import frc.robot.constants.Constants.Shooter;

/**
 * Interface class for a shooter flywheel. <p>
 * Uses two linked TalonFX controlled motors. <p>
 * Uses MotionMagic to control velocity.
 * @author 5985
 */
public class Flywheels 
{
  private final TalonFX m_Leader; 
  private final TalonFX m_Follower;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

  /**
   * Creates a velocity controlled flywheel, to be managed by {@link Shooter} master-system
   * @param leaderCAN CAN-ID of primary shooter motor
   * @param followerCAN CAN-ID of secondary shooter motor, set to follow first
   */
  public Flywheels(int leaderCAN, int followerCAN)
  {
    m_Leader = new TalonFX(leaderCAN);
    m_Follower = new TalonFX(followerCAN);

    var shooterConfigs = flywheelConfig;

    m_Leader.getConfigurator().apply(shooterConfigs);

    m_Follower.setControl(new Follower(leaderCAN, MotorAlignmentValue.Opposed));
  }

  /**
   * Set the motor speed
   * 
   * @param speed the desired speed, in mechanism rotations per second
   */
  public void setSpeed(double speed)
    {m_Leader.setControl(request.withVelocity(speed));}

  /**
   * Checks if the current motor speed is within {@link Shooter#flySpeedTolerance flySpeedTolerance} of the requested speed
   * 
   * @return true if the motor is at speed
   */
  public boolean atSpeed() 
  {
    double currentSpeed = m_Leader.getVelocity().getValueAsDouble();
    return MathUtil.isNear(request.Velocity, currentSpeed, flySpeedTolerance);
  }
}
