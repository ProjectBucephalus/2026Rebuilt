package frc.robot.controlTransmutation.triggerObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controlTransmutation.FieldObject;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/** 
 * Derived from GeoFence logic, acts as a position/distance check and trigger <p>
 * Can add an optional non-directional speed-limit within the area
 * @author 5985
 */
public class TriggerRegion extends FieldObject
{
  protected double localSpeedLimit = 0;

  /**
   * Circle shaped region
   * @param x X-coordinate of the centre point
   * @param y Y-coordinate of the centre point
   * @param radius Radius of the circle (0 for point)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public TriggerRegion(double x, double y, double radius, double buffer)
  {
    centre = new Translation2d(x, y);
    this.radius = Math.max(radius, minRadius);
    this.buffer = Math.max(buffer, minBuffer);

    checkRadius = radius + buffer;
  }

  /**
   * Circle shaped region
   * @param centre The centre point
   * @param radius Radius of the circle (0 for point)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public TriggerRegion(Translation2d centre, double radius, double buffer)
  {
    this.centre = centre;
    this.radius = Math.max(radius, minRadius);
    this.buffer = Math.max(buffer, minBuffer);

    checkRadius = radius + buffer;
  }

  /** Default constructor for fully zeroed circle-shaped region */
  public TriggerRegion()
    {this(0, 0, 0, 0);}

  /**
   * Sets the speed limit within the region <p>
   * Set the speed limit to zero to use the object as a position/distance check
   * @param localSpeedLimit The new speed limit value
   * @return This region, for easier chaining
   */
  public TriggerRegion withSpeedLimit(double localSpeedLimit)
  {
    this.localSpeedLimit = localSpeedLimit >= minLocalSpeedLimit ? localSpeedLimit : 0;
    return this;
  }

  /** @return A trigger for whether the robot is within the region */
  public Trigger asTrigger()
    {return new Trigger(activeSupplier).and(globalActiveSupplier).and(() -> checkPosition() && getDistance() <= 0);}

  /**
   * Caps the maximum speed to the configured speed limit if within the region,
   * or an intermediate speed proportional to the distance from the region if within the buffer zone
   * 
   * @return Speed-limited joystick output [-limit..limit],[-limit..limit]
   */
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    if 
    (
      globalActiveSupplier.getAsBoolean() &&
      activeSupplier.getAsBoolean() && 
      localSpeedLimit > 0 && 
      checkPosition() && 
      !controlInput.equals(Translation2d.kZero)
    )
    {
      double motionNormal = controlInput.getNorm();
      Rotation2d motionAngle = controlInput.getAngle();

      double distance = getDistance();

      if (distance <= 0)
      {
        return new Translation2d(Math.min(motionNormal, localSpeedLimit), motionAngle);
      }

      if (distance <= buffer)
      {
        return new Translation2d(Math.min(motionNormal, MathUtil.interpolate(1, localSpeedLimit, distance/buffer)), motionAngle);
      }
    }

    return controlInput;
  }
}
