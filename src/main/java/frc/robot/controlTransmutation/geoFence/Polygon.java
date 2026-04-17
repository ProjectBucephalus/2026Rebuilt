package frc.robot.controlTransmutation.geoFence;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/**
 * A polygon shaped {@link GeoFence} <p>
 * A rotated regular polygon built from a series of {@link Line Lines}
 * with handling to only process the nearest line
 * @author 5985
 */
public class Polygon extends GeoFence
{
  private Line[] edgeLines;

  /**
    * Define regular polygon object
    * @param X x-coordinate of centre, metres
    * @param Y y-coordinate of centre, metres
    * @param buffer range over which the robot slows down, metres  
    * @param radius circumscribed (centre-corner) radius of the object, metres
    * @param theta angle of the object: 0 = "corner at North", degrees Anticlockwise
    * @param sides number of polygon sides, integer [3..12]
    */
  public Polygon(double x, double y, double radius, double buffer, double theta, int sides)
  {
    // Constraining inputs
    radius = Math.max(radius, minRadius);
    buffer = Math.max(buffer, minBuffer);
    sides = Conversions.clamp(sides, 3, 12);

    // Initialising instance variables
    this.centre = new Translation2d(x, y);
    this.checkRadius = radius + buffer;
    this.edgeLines = new Line[sides];

    // The start of the first line/end of the last line
    var initalPoint = new Translation2d(x, y + radius).rotateAround(centre, Rotation2d.fromDegrees(theta));

    // Line endpoints are equidistant around a circle
    var pointAngle = Rotation2d.fromDegrees(360.0 / (sides * 2));

    // Starting with the initial point, each subsequent point is equal to the previous point rotated by the angle
    // Extra point between each line terminator point for the line midpoint
    var polygonPoints = Stream
      .iterate(initalPoint, prev -> prev.rotateAround(centre, pointAngle))
      .limit(sides * 2 + 1l)
      .toList();
    
    for (int i = 0; i < sides; i++)
    {
      edgeLines[i] = new Line
      (
        polygonPoints.get(2 * i),
        polygonPoints.get(2 * i + 2),
        0,
        buffer
      );
    }
      
    /* 
    * Convert the circumscribed radius (centre-corner) to the inscribed radius (centre-edge)
    * and expand the buffer to account for the difference
    * 
    * These values are used to process the polygon as a point if the robot crosses the lines
    */ 
    this.radius = pointAngle.getCos() * radius;
    this.buffer = buffer + (radius - this.radius);
  }

  @Override
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    Line processLine = nearestLine();
    // If the robot is inside the polygon, process based on the inscribed circle
    if (processLine.getDirectionalDistance() < 0)
      return pointDamping(centre.getX(), centre.getY(), motionXY);
    else 
      /* 
      * Damps the motion based on the line closest to the robot:
      * Polygon objects consist of a list of lines and a list of reference points
      * Finding the index of the closest reference point gives the index of the closest line
      */
      return processLine.dampMotion(motionXY);
  }

  @Override
  public double getDistance()
    {return nearestLine().getDirectionalDistance();}

  @Override
  public boolean checkAttractors() 
    {return nearestLine().checkAttractors();}

  /** Processes the attractors of the 3 closest lines in increasing distance order, returning the first that changes the input */
  @Override
  public Translation2d processAttractors(Translation2d controlInput) 
  {
    int nearestIndex = nearestLineIndex();

    Translation2d controlOutput = edgeLines[nearestIndex].processAttractors(controlInput);
    if (!controlOutput.equals(controlInput)) {return controlOutput;}

    controlOutput = edgeLines[Conversions.wrap(nearestIndex - 1, 0, edgeLines.length - 1)].processAttractors(controlInput);
    if (!controlOutput.equals(controlInput)) {return controlOutput;}

    controlOutput = edgeLines[Conversions.wrap(nearestIndex + 1, 0, edgeLines.length - 1)].processAttractors(controlInput);
    if (!controlOutput.equals(controlInput)) {return controlOutput;}

    return controlInput;
  }

  /** @return The nearest line of the polygon */
  private Line nearestLine()
  {
    return Arrays
      .stream(edgeLines)
      .min(Comparator.comparingDouble(line -> line.getCentre().getDistance(robotPos)))
      .orElseThrow();
  }

  /** @return The index of the nearest line of the polygon */
  private int nearestLineIndex()
  {
    return IntStream.range(0, edgeLines.length)
      .boxed()
      .min(Comparator.comparingDouble(idx -> edgeLines[idx].getCentre().getDistance(robotPos)))
      .orElseThrow();
  }

  /**
   * Gets the list of midpoints of the lines of the polygon, followed by the centre of the polygon
   * @return List of Translation2ds, metres
   */
  public List<Translation2d> getMidPoints()
    {return Arrays.stream(edgeLines).map(Line::getCentre).toList();}

  /**
   * Constructs and adds an Attractor on each face of the Polygon
   * @param normalOffset Distance away from the line along the approach direction, metres
   * @param tangentOffset Distance away from the line centre, metres right relative to the approach direction
   * @param effectRadius Distance at which the Attractor becomes active, metres
   * @param targetBuffer Distance at which the Robot must be moving along the approach direction, metres
   * @param activeCondition Condition for which the Attractor is active
   * @return The Polygon object with the new Attractor
   */
  public Polygon addRelativeAttractors(double normalOffset, double tangentOffset, double effectRadius, double targetBuffer, BooleanSupplier activeCondition)
  {
    for (var edge : edgeLines)
      edge.addRelativeAttractor(false, normalOffset, tangentOffset, effectRadius, targetBuffer, activeCondition);
    return this;
  }
}