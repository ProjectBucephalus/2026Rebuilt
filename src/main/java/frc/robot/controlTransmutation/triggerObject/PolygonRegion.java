package frc.robot.controlTransmutation.triggerObject;

import java.util.Arrays;
import java.util.Comparator;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/** 
 * A polygon shaped region
 * @author 5985
 */
public class PolygonRegion extends TriggerRegion 
{
  private LineRegion[] polygonLines;
  
  /**
   * Polygon shaped region
   * @param x X-coordinate of the centre point
   * @param y Y-coordinate of the centre point
   * @param radius Extra radius of region around the polygon (produces a rounded polgyon shape)
   * @param buffer Buffer around the object over which the speed is reduced
   * @param theta Rotation of the polygon. Zero means a point will be facing north
   * @param sides Side count of the polygon
   */
  public PolygonRegion(double x, double y, double radius, double buffer, double theta, int sides)
  {
    // Constraining inputs
    radius = Math.max(radius, minRadius);
    buffer = Math.max(buffer, minBuffer);
    sides = Conversions.clamp(sides, 3, 12);

    // Initialising instance variables
    this.centre = new Translation2d(x, y);
    this.checkRadius = radius + buffer;
    this.polygonLines = new LineRegion[sides];

    // The start of the first line/end of the last line
    var initalPoint = new Translation2d(x, y + radius).rotateAround(centre, Rotation2d.fromDegrees(theta));

    // Line endpoints are equidistant around a circle
    var pointAngle = Rotation2d.fromDegrees(360.0 / sides);

    // Starting with the initial point, each subsequent point is equal to the previous point rotated by the angle
    var polygonPoints = Stream
      .iterate(initalPoint, prev -> prev.rotateAround(centre, pointAngle))
      .limit(sides + 1l)
      .toList();
    
    for (int i = 0; i < sides; i++)
      polygonLines[i] = new LineRegion(polygonPoints.get(i), polygonPoints.get(i + 1));

    /* 
    * Convert the circumscribed radius (centre-corner) to the inscribed radius (centre-edge)
    * and expand the buffer to account for the difference
    * 
    * These values are used to process the polygon as a point if the robot crosses the lines
    */ 
    this.radius = pointAngle.getCos() * radius;
    this.buffer = buffer + (radius - this.radius);
  }

  /**
   * {@inheritDoc} <p>
   * Distance is based on the distance to the nearest line
   */
  @Override
  public double getDistance()
    {return nearestLine().getDirectionalDistance();}

  /** @return The nearest line of the polygon */
  private LineRegion nearestLine()
  {
    return Arrays
      .stream(polygonLines)
      .min(Comparator.comparingDouble(line -> line.getCentre().getDistance(robotPos)))
      .orElseThrow();
  }
}
