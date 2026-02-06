package frc.robot.util.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/**
 * Point type GeoFence object </p>
 * Defined as a single point with a radius
 * @author 5985
 */
public class Point extends GeoFence
{
  public Point(double x, double y, double radius, double buffer)
  {
    centre = new Translation2d(x, y);
    this.radius = Math.max(radius, minRadius);
    this.buffer = Math.max(buffer, minBuffer);

    checkRadius = radius + buffer;
  }

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