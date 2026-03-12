package frc.robot.util.controlTransmutation.restrictor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.controlTransmutation.FieldObject;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/** 
 * Derived from GeoFence logic, acts as a non-directional speed-limit within the given area <p>
 * Using a local speed limit <= 0 allows it to be used as a position/distance check <p>
 * Can also be used as a trigger determining whether the robot is within the restrictor
 * @author 5985
 */
public class Restrictor extends FieldObject
{
  private final Trigger trigger = new Trigger(() -> checkPosition() && getDistance() <= 0);
  protected double localSpeedLimit = 0;

  /**
   * Circle shaped restrictor
   * @param x X-coordinate of the centre point
   * @param y Y-coordinate of the centre point
   * @param radius Radius of the circle (0 for point)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public Restrictor(double x, double y, double radius, double buffer)
  {
    centre = new Translation2d(x, y);
    this.radius = Math.max(radius, minRadius);
    this.buffer = Math.max(buffer, minBuffer);

    checkRadius = radius + buffer;
  }

  /**
   * Circle shaped restrictor
   * @param centre The centre point
   * @param radius Radius of the circle (0 for point)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public Restrictor(Translation2d centre, double radius, double buffer)
  {
    this.centre = centre;
    this.radius = Math.max(radius, minRadius);
    this.buffer = Math.max(buffer, minBuffer);

    checkRadius = radius + buffer;
  }

  /** Default constructor for fully zeroed circle-shaped restrictor */
  public Restrictor()
    {this(0, 0, 0, 0);}

  /**
   * Sets the speed limit within the restrictor <p>
   * Set the speed limit to zero to use the object as a position/distance check
   * @param localSpeedLimit The new speed limit value
   * @return This restrictor, for easier chaining
   */
  public Restrictor withSpeedLimit(double localSpeedLimit)
  {
    this.localSpeedLimit = localSpeedLimit >= minLocalSpeedLimit ? localSpeedLimit : 0;
    return this;
  }

  /** @return A trigger for whether the robot is within the restrictor zone */
  public Trigger asTrigger()
    {return trigger;}

  /**
   * Caps the maximum speed to the configured speed limit if within the restrictor,
   * or an intermediate speed proportional to the distance from the restrictor if within the buffer zone
   * 
   * @return Speed-limited joystick output [-limit..limit],[-limit..limit]
   */
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    if 
    (
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
