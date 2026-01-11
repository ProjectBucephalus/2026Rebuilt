// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation.geoFence;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.controlTransmutation.GeoFence;
import static frc.robot.constants.FieldConstants.GeoFencing.*;

/**
 * Polygon type GeoFence object </p>
 * A rotated regular polygon built from a series of Line objects </p>
 * with handling to only process the nearest line
 */
public class Polygon extends GeoFence
{
  List<Line> edgeLines;
  List<Translation2d> edgeReference;
  int sides;

  /**
    * Define regular polygon object
    * @param X x-coordinate of centre, metres
    * @param Y y-coordinate of centre, metres
    * @param buffer range over which the robot slows down, metres  
    * @param radius circumscribed (centre-corner) radius of the object, metres
    * @param theta angle of the object: 0 = "corner at North", degrees Anticlockwise
    * @param sides number of polygon sides, integer [3..12]
    */
  public Polygon(double X, double Y, double radius, double buffer, double theta, int sides)
  {
    edgeLines = new ArrayList<Line>();
    edgeReference = new ArrayList<Translation2d>();

    centre = new Translation2d(X,Y);

    // Constraining inputs
    radius = Math.max(Math.abs(radius), minRadius);
    buffer = Math.max(buffer, minBuffer);
    sides = Conversions.clamp(sides, 3, 12);
    
    // Array of all points to construct the polygon lines and references
    // The start of the first line and end of the last line are separate enteries to simplify construction
    Translation2d[] polygonPoints = new Translation2d[2*sides+1];

    polygonPoints[0] = new Translation2d(X,Y + radius).rotateAround(centre, Rotation2d.fromDegrees(theta));

    // Line endpoints and reference points are equidistant around a circle
    Rotation2d rotationBetweenPoints = Rotation2d.fromDegrees(360/(2*sides));
    for (int i = 1; i < polygonPoints.length; i++)
      {polygonPoints[i] = polygonPoints[i-1].rotateAround(centre, rotationBetweenPoints);}

    for (int i = 0; i < sides; i++)
    {
      edgeLines.add(i, new Line
      (
        polygonPoints[2*i].getX(),
        polygonPoints[2*i].getY(),
        polygonPoints[2*i+2].getX(),
        polygonPoints[2*i+2].getY(),
        0,
        buffer
      ));

      edgeReference.add(i, polygonPoints[2*i+1]);
    }

    /* 
      * Convert the circumscribed radius (centre-corner) to the inscribed radius (centre-edge)
      * and expand the buffer to account for the difference
      * 
      * These values are used to process the polygon as a point if the robot crosses the lines
      */ 
    this.radius = rotationBetweenPoints.getCos() * radius;
    this.buffer = buffer + (radius - this.radius);

    checkRadius = radius + buffer;
  }

  @Override
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    Line processLine = nearestLine();
    // If the robot is inside the polygon, process based on the inscribed circle
      if (processLine.getDirectionalDistance() < 0)
        {return pointDamping(centre.getX(), centre.getY(), motionXY);}
  
    /* 
      * Damps the motion based on the line closest to the robot:
      * Polygon objects consist of a list of lines and a list of reference points
      * Finding the index of the closest reference point gives the index of the closest line
      */
    return processLine.dampMotion(motionXY);
  }

  @Override
  public double getDistance()
  {
    return nearestLine().getDirectionalDistance();
  }

  @Override
  public boolean checkAttractors() 
  {
    return nearestLine().checkAttractors();
  }

  @Override
  public Translation2d processAttractors(Translation2d controlInput) 
  {
    int nearestIndex = nearestLineIndex();
    Translation2d controlOutput = edgeLines.get(nearestIndex).processAttractors(controlInput);
    if (!controlOutput.equals(controlInput)) {return controlOutput;}
    controlOutput = edgeLines.get(Conversions.wrap(nearestIndex - 1, 0, edgeLines.size() - 1)).processAttractors(controlInput);
    if (!controlOutput.equals(controlInput)) {return controlOutput;}
    controlOutput = edgeLines.get(Conversions.wrap(nearestIndex + 1, 0, edgeLines.size() - 1)).processAttractors(controlInput);
    if (!controlOutput.equals(controlInput)) {return controlOutput;}
    return controlInput;
  }

  private Line nearestLine()
  {
    int index = 0;
    double minDistance = edgeLines.get(0).getCentre().getDistance(robotPos);
    double checkDistance;

    for (int i = 1; i < edgeLines.size(); i++)
    {
      checkDistance = edgeLines.get(i).getCentre().getDistance(robotPos);
      if (checkDistance < minDistance)
      {
        index = i;
        minDistance = checkDistance;
      }
    }

    return edgeLines.get(index);
  }

  private int nearestLineIndex()
  {
    int index = 0;
    double minDistance = edgeLines.get(0).getCentre().getDistance(robotPos);
    double checkDistance;

    for (int i = 1; i < edgeLines.size(); i++)
    {
      checkDistance = edgeLines.get(i).getCentre().getDistance(robotPos);
      if (checkDistance < minDistance)
      {
        index = i;
        minDistance = checkDistance;
      }
    }

    return index;
  }

  /**
   * Gets the list of midpoints of the lines of the polygon, followed by the centre of the polygon
   * @return List of Translation2ds, metres
   */
  public ArrayList<Translation2d> getMidPoints()
  {
    ArrayList<Translation2d> midPoints = new ArrayList<Translation2d>();

    for (int i = 0; i < edgeLines.size(); i++)
    {
      midPoints.add(edgeLines.get(i).getCentre());
    }

    midPoints.add(centre);
    return midPoints;
  }

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
    for (int i = 0; i < edgeLines.size(); i++)
    {
      edgeLines.get(i).addRelativeAttractor(false, normalOffset, tangentOffset, effectRadius, targetBuffer, activeCondition);
    }
    return this;
  }
}