// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;

/** Modified deadband filter that locks the input to cardinal outputs */
public class CrossDeadband extends Deadband
  {
    protected double overlap;
    
    /**
     * Snaps the input to be purely cardinal
     * @param deadband Size of centre deadband
     * @param overlap Determines the size and behaviour of corners: <1 deadzone, >1 smooth control, default 1 direct change from X to Y
     */
    public CrossDeadband()
    {
      this(Constants.Control.stickDeadband, 1);
    }

    /**
     * Snaps the input to be purely cardinal
     * @param deadband Size of centre deadband
     * @param overlap Determines the size and behaviour of corners: <1 deadzone, >1 smooth control, (default) 1 direct change from X to Y
     */
    public CrossDeadband(double deadband, double overlap)
    {
      super.deadband = deadband;
      this.overlap = overlap;
    }

    @Override
    public Translation2d process(Translation2d controlInput)
    {
      if (controlInput.getNorm() <= deadband)
        {return Translation2d.kZero;}
        
      return new Translation2d
      (
        Math.abs(controlInput.getX()) < overlap * Math.abs(controlInput.getY()) ? 0 : controlInput.getX(),
        Math.abs(controlInput.getY()) < overlap * Math.abs(controlInput.getX()) ? 0 : controlInput.getY()  
      );
    }
  }
