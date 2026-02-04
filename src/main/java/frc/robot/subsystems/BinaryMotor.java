// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * A subsystem wrapped around a TalonFX to provide a simple subsystem for any motor mainly intended for binary operation (on or off) 
 * @author 5985
 */
public class BinaryMotor extends SubsystemBase 
{
  private final TalonFX m_Inner;
  private final double defaultSpeed;

  /**
   * Creates a wrapper around a TalonFX to provides simple binary control (on or off)
   * 
   * @param defaultSpeed the duty-cycle speed to run at when on [-1..1]
   * @param id the id of the motor
   */
  public BinaryMotor(int id, double defaultSpeed) 
  {
    this.defaultSpeed = defaultSpeed;
    m_Inner = new TalonFX(id);
  }

  public BinaryMotor(int id, double defaultSpeed, TalonFXConfiguration config) 
  {
    this.defaultSpeed = defaultSpeed;
    m_Inner = new TalonFX(id);
    applyConfig(config);
  }

  public BinaryMotor applyConfig(TalonFXConfiguration config) 
  {
    m_Inner.getConfigurator().apply(config);
    return this;
  }

  /**
   * Construct a command that runs the motor at the default speed
   * 
   * @return the {@link Command}
   */
  public Command startCommand()
    {return runOnce(() -> m_Inner.set(defaultSpeed));}

  /**
   * Construct a command that stops the motor (i.e., sets speed to 0) 
   * 
   * @return the {@link Command}
   */
  public Command stopCommand()
    {return runOnce(() -> m_Inner.set(0));}

  /**
   * Construct a command that sets the speed for the motor to an arbitrary value <p>
   * NOTE: The provided value is only evaluated when the command is created
   * 
   * @param speed the duty-cycle speed to run at [-1..1]
   * @return the {@link Command}
   */
  public Command setSpeedCommand(double speed)
    {return runOnce(() -> m_Inner.set(speed));}
}
