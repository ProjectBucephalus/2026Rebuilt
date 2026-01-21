// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.constants.Constants.FeedBeltConstants.*;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.BinaryMotor;

/** Add your docs here. */
public class FeedBelt extends BinaryMotor
{
  public FeedBelt()
  {
    super(speed, IDConstants.beltID);
  }

}
