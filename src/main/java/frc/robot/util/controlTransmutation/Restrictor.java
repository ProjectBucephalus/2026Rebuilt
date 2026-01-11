package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/** 
 * Derived from GeoFence logic, acts as a non-directional speed-limit within the given area </p>
 * Using a local speed limit <= 0 allows it to be used as a position/distance check
 */
public class Restrictor extends FieldObject
{
  protected double localSpeedLimit;

  /**
   * Basic point-type restrictor object
   * @param x X-coordinate of the point
   * @param y Y-coordinate of the point
   * @param radius Radius of the circle (0 for point)
   * @param buffer Buffer around the object over which the speed is reduced
   * @param localSpeedLimit Maximum speed when fully in the area (0.05..1] </p>
   * Set localSpeedLimit = 0 to use the object as a position/distance check
   */
  public Restrictor(double x, double y, double radius, double buffer, double localSpeedLimit)
  {
    centre = new Translation2d(x, y);
    this.radius = Math.max(radius, minRadius);
    this.buffer = Math.max(buffer, minBuffer);
    this.localSpeedLimit = localSpeedLimit >= minLocalSpeedLimit ? localSpeedLimit : 0;

    checkRadius = radius + buffer;
  }

  /**
   * Basic point-type restrictor object
   * @param x X-coordinate of the point
   * @param y Y-coordinate of the point
   * @param radius Radius of the circle (0 for point)
   * @param buffer Buffer around the object over which the speed is reduced
   * @param localSpeedLimit Maximum speed when fully in the area (0.05..1] </p>
   * Set localSpeedLimit = 0 to use the object as a position/distance check
   */
  public Restrictor()
    {this(0, 0, 0, 0, 0);}

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
