package frc.robot.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.ControlConstants;

/** 
 * Applies a modified deadband that locks the input to cardinal outputs 
 * @author 5985
 */
public class CrossDeadband extends Deadband
{
  protected double overlap;
  
  /**
   * Snaps the input to be purely cardinal
   * @param deadbandVal Optional, absolute value of input below which the output will be zero. Defaults to {@link ControlConstants#stickDeadband stickDeadband}
   * @param overlap Determines the size and behaviour of corners: <1 deadzone, >1 smooth control, (default) 1 direct change from X to Y
   */
  public CrossDeadband()
  {
    this(ControlConstants.stickDeadband, 1);
  }

  /**
   * Snaps the input to be purely cardinal
   * @param deadband Size of centre deadband
   * @param overlap Determines the size and behaviour of corners: <1 deadzone, >1 smooth control, (default) 1 direct change from X to Y
   */
  public CrossDeadband(double deadband, double overlap)
  {
    super(deadband);
    this.overlap = overlap;
  }

  /**
   * Zeroes the input if its normal is below the threshold,
   * otherwise adjusts the input to be cardinally-locked (or biased, depending on specified overlap)
   */
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    if (controlInput.getNorm() <= deadbandVal)
      {return Translation2d.kZero;}
      
    return new Translation2d
    (
      Math.abs(controlInput.getX()) < overlap * Math.abs(controlInput.getY()) ? 0 : controlInput.getX(),
      Math.abs(controlInput.getY()) < overlap * Math.abs(controlInput.getX()) ? 0 : controlInput.getY()  
    );
  }
}
