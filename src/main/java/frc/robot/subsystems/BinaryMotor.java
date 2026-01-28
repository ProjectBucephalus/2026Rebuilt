// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class BinaryMotor extends SubsystemBase 
{
  private final double defaultSpeed;
  private final TalonFX m_Inner;

  public BinaryMotor(double defaultSpeed, int id) 
  {
    this.defaultSpeed = defaultSpeed;
    m_Inner = new TalonFX(id);
  }

  public Command startCommand()
    {return runOnce(() -> m_Inner.set(defaultSpeed));}

  public Command stopCommand()
    {return runOnce(() -> m_Inner.set(0));}

  public Command setSpeedCommand(double speed)
    {return runOnce(() -> m_Inner.set(speed));}
}
