// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;

/** Deadband region for the input */
public class Deadband extends InputTransmuter
{
  protected double deadband;

  /**
   * Creates a deadband filter to zero any inputs below the given threshold
   * @param deadband Optional, absolute value of input below which the output will be zero. Defaults to Constants.Control.stickDeadband
   */
  public Deadband()
    {this(Constants.Control.stickDeadband);}
  
  /**
   * Creates a deadband filter to zero any inputs below the given threshold
   * @param deadband Optional, absolute value of input below which the output will be zero
   */
  public Deadband(double deadband)
    {this.deadband = deadband;}
  
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    return (controlInput.getNorm() <= deadband ? Translation2d.kZero : controlInput);
  }
}