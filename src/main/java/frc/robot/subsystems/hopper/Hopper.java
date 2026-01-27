// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.Constants.HopperConstants.*;

import frc.robot.subsystems.BinaryMotor;
import frc.robot.subsystems.LinearExtension;

public class Hopper extends SubsystemBase {
  private BinaryMotor spindexer;
  private BinaryMotor intake;
  private LinearExtension extension;
  
  /** Creates a new Hopper. */
  public Hopper(int spindexerCAN, int intakeCAN, int extensionCAN, int extensionLimitCAN) 
  {
    spindexer = new BinaryMotor(spindexerSpeed, spindexerCAN);
    intake = new BinaryMotor(intakeSpeed, intakeCAN);
    extension = new LinearExtension(extensionCAN, extensionLimitCAN, 0, ExtensionConstants.maxRotations, ExtensionConstants.config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
