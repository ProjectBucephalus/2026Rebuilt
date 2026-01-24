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

  // creates a new fuunction that other subsystems can call
  public LinearExtension(int motorID, int limitID, double minRotations, double maxRotations, TalonFXConfiguration configs)
  {
    this.maxRotations = maxRotations;
    this.minRotations = minRotations;
    slot1Valid = configs.Slot1.kP != 0;

    m_Extension = new TalonFX(motorID);
    io_Limit = new DigitalInput(limitID);

    m_Extension.getConfigurator().apply(configs);

    m_Extension.setPosition(maxRotations);
  } 

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
  public void periodic() 
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
