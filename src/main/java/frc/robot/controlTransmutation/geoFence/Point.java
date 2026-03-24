package frc.robot.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/** 
 * A circle/point shaped {@link GeoFence} <p>
 * Defined as a single point with a radius
 * @author 5985
 */
public class Point extends GeoFence
{
  /**
   * Circle/point shaped GeoFence
   * @param x X-coordinate of the centre point
   * @param y Y-coordinate of the centre point
   * @param radius Radius of the circle (0 for point)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public Point(double x, double y, double radius, double buffer)
  {
    centre = new Translation2d(x, y);
    this.radius = Math.max(radius, minRadius);
    this.buffer = Math.max(buffer, minBuffer);

    checkRadius = radius + buffer;
  }

  /**
   * Point shaped GeoFence with minimum radius and buffer
   * @param x X-coordinate of the centre point
   * @param y Y-coordinate of the centre point
   */
  public Point(double x, double y)
  {
    this(x, y, minRadius, minBuffer);
  }

  @Override
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    return pointDamping(centre.getX(), centre.getY(), motionXY);
  }
}