// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.HopperConstants.ExtensionConstants.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Conversions;


/** Add your docs here. */
public class LinearExtension extends SubsystemBase
{
  private TalonFX m_Extension;
  private DigitalInput io_Limit;
  private final double maxRotations;
  private final double minRotations;
  private boolean homed = false;
  private boolean homeLastCycle = false;
  private final boolean slot1Valid;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  /**
   * Creates generic linear extension system
   * @param motorCAN CAN-ID of extension motor
   * @param limitIO DIO-ID of home limit-sensor
   * @param minRotations Minimum position in mechanism rotations
   * @param maxRotations Maximum position in mechanism rotations
   * @param configs Motor configuration object, uses Slot1 if present when not calibrated
   */
  public LinearExtension(int motorCAN, int limitIO, double minRotations, double maxRotations, TalonFXConfiguration configs)
  {
    this.maxRotations = maxRotations;
    this.minRotations = minRotations;
    slot1Valid = configs.Slot1.kP != 0;

    m_Extension = new TalonFX(motorCAN);
    io_Limit = new DigitalInput(limitIO);

    m_Extension.getConfigurator().apply(configs);

    m_Extension.setPosition(maxRotations);
  } 

  /**
   * Creates a command to set the target point for the extension
   * NOTE: The provided value is only evaluated when the command is created
   * @param targetRotations
   * @return
   */
  public Command setTargetCommand(double targetRotations)
  {
    return runOnce(() -> {
      double clampedRotations = Conversions.clamp(targetRotations, minRotations, maxRotations);
      int slot = !homed && slot1Valid ? 1 : 0;
      m_Extension.setControl
        (request.withPosition(clampedRotations).withSlot(slot));
    });
  }
  
  @Override
  public void periodic() // TODO This is causing loop overruns???
  {
    if (io_Limit.get())
      if (!homeLastCycle)
      {
        homed = true;
        homeLastCycle = true;
        m_Extension.setPosition(0);
      }
      else 
        homeLastCycle = false;
  }
}
