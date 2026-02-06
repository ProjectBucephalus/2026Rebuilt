package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.ControlConstants;;

/** 
 * Applies a deadband region to the input 
 * @author 5985
 */
public class Deadband implements InputTransmuter
{
  protected double deadband;

  /**
   * Creates a deadband filter to zero any inputs below a threshold
   * @param deadband Optional, absolute value of input below which the output will be zero. Defaults to {@link ControlConstants#stickDeadband stickDeadband}
   */
  public Deadband()
    {this(ControlConstants.stickDeadband);}
  
  /**
   * Creates a deadband filter to zero any inputs below a threshold
   * @param deadband Optional, absolute value of input below which the output will be zero. Defaults to {@link ControlConstants#stickDeadband stickDeadband}
   */
  public Deadband(double deadband)
    {this.deadband = deadband;}
  
  /** Zeroes the input if its normal is below the threshold */
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    return (controlInput.getNorm() <= deadband ? Translation2d.kZero : controlInput);
  }
}