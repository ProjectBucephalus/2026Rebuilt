// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.Constants.HopperConstants.*;
import frc.robot.subsystems.BinaryMotor;


public class Hopper extends SubsystemBase {
  private BinaryMotor feedBelt;
  private BinaryMotor intake;
  
  /** Creates a new Hopper. */
  public Hopper(int feedBeltID, int intakeID) 
  {
    feedBelt = new BinaryMotor(beltSpeed, feedBeltID);
    intake = new BinaryMotor(intakeSpeed, intakeID);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
