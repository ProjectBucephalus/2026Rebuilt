package frc.robot.controlTransmutation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.util.Conversions;

/** 
 * Throttle modifier for the input 
 * @author 5985
 */
public class Brake implements InputTransmuter
{
  private DoubleSupplier brakeAxis;
  /** Maximum throttle when the brake is fully released, [0..1] */
  private double max;
  /** Minimum throttle when the brake is fully pressed, [0..1] */
  private double min;

  /**
   * Creates a brake filter
   * @param brakeAxis DoubleSupplier for the brake axis
   * @param maxThrottle Maximum throttle when the brake is fully released, [0..1]
   * @param minThrottle Minimum throttle when the brake is fully pressed, [0..1]
   */
  public Brake()
  {
    brakeAxis = null;
    max = Constants.ControlConstants.maxThrottle;
    min = Constants.ControlConstants.minThrottle;
  }

  /**
   * Creates a brake filter
   * @param brakeAxis DoubleSupplier for the brake axis
   * @param maxThrottle Maximum throttle when the brake is fully released, [0..1]
   * @param minThrottle Minimum throttle when the brake is fully pressed, [0..1]
   */
  public Brake(DoubleSupplier brakeAxis, double maxThrottle, double minThrottle)
  {
    this.brakeAxis = brakeAxis;
    max = maxThrottle;
    min = minThrottle;
  }

  /**
   * Gets the scaled value of the brake axis
   * @return Brake scale, [minThrottle..maxThrottle]
   */
  public double get()
  {
    return brakeAxis == null ? max : MathUtil.interpolate(max, min, brakeAxis.getAsDouble());
  }

  @Override
  public Translation2d process(Translation2d controlInput)
  {
    return controlInput.times(get());
  }

  /**
   * Sets the brake axis
   * @param brakeAxis DoubleSupplier for the new brake axis
   * @return The brake object with the new axis
   */
  public Brake withBrakeAxis(DoubleSupplier brakeAxis)
  {
    this.brakeAxis = brakeAxis;
    return this;
  }

  /**
   * Sets the maximum throttle value with no braking
   * @param newMax new maximum throttle value, [0..1]
   * @return The brake object with the new value
   */
  public Brake withMaxThrottle(double newMax)
  {
    max = Conversions.clamp(newMax, 0, 1);
    return this;
  }

  /**
   * Sets the minimum throttle value under full brake
   * @param newMax new maximum throttle value, [0..1]
   * @return The brake object with the new value
   */
  public Brake withMinThrottle(double newMin)
  {
    min = Conversions.clamp(newMin, 0, 1);
    return this;
  }
}