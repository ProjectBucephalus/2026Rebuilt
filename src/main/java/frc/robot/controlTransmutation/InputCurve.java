package frc.robot.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;

/** 
 * Applies a parabolic sensitivity curve to the input 
 * @author 5985 
 */
public class InputCurve implements InputTransmuter
{
  private double power;

  /** 
   * Parabolic curve on axis input
   * @param power optional, defaults to minimum 1 (linear)
   */
  public InputCurve()
    {this(1);}
  
  /** 
   * Parabolic curve on axis input
   * @param power optional, defaults to minimum 1 (linear)
   */
  public InputCurve(double power)
    {this.power = Math.max(power, 1);}

  /**
   * Applies a sensitivity curve to the input, keeping the range from [0..1] but causing it to scale faster the closer it gets to 1 <>
   * Allows finer control at slow speeds while maintaining the same top speedp
   */
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    double norm = controlInput.getNorm();
    double scale = Math.pow(norm, power - 1);
    return controlInput.times(scale);

    // return Conversions.clamp
    // (
    //   new Translation2d
    //   (
    //     Math.copySign(Math.pow(controlInput.getX(), power), controlInput.getX()), 
    //     Math.copySign(Math.pow(controlInput.getY(), power), controlInput.getY())
    //   )
    // );
  }
}