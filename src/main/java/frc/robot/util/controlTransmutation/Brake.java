// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;

/** Throttle modifier for the input */
public class Brake extends InputTransmuter
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
    max = Constants.Control.maxThrottle;
    min = Constants.Control.minThrottle;
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
}