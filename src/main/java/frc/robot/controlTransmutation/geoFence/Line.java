package frc.robot.controlTransmutation.geoFence;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.controlTransmutation.triggerObject.TriggerVector;
import frc.robot.util.Conversions;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/**
 * A line shaped {@link GeoFence} </p>
 * Defined between two points </p>
 * Note: causes edge-case behaviours when meeting other objects at acute angles
 * @author 5985
 */
public class Line extends GeoFence
{
  private Translation2d pointA;
  private Translation2d pointB;
  private double dXab;
  private double dYab;
  private double length;
  private double dotX;
  private double dotY;
  private double dotXY;
  private double normX;
  private double normY;
  private double normXY;

  /**
   * Line shaped GeoFence <p>
   * When looking along the line from point A to B, right is positive distance, left is negative distance
   * @param pointA The first point
   * @param pointB The second point
   * @param radius Extra radius of geofence zone around the line (produces a capsule shape)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public Line(Translation2d pointA, Translation2d pointB, double radius, double buffer)
  {
    this.radius = radius;
    this.buffer = buffer;
    this.pointA = pointA;
    this.pointB = pointB;

    // Centre is halfway along the line
    centre = new Translation2d((pointA.getX() + pointB.getX()) / 2, (pointA.getY() + pointB.getY()) / 2);

    dXab = pointB.getX() - pointA.getX();
    dYab = pointB.getY() - pointA.getY();
    length = Math.hypot(dXab, dYab);
    
    normX = dXab / length;
    normY = dYab / length;
    normXY = ((pointA.getX() * pointB.getY()) - (pointB.getX() * pointA.getY())) / length;
    
    dotX = normX / length;
    dotY = normY / length;
    dotXY = (pointA.getX() * dotX) + (pointA.getY() * dotY);

    checkRadius = (Math.sqrt(length)/2) + radius + buffer;
  }

  /**
   * Line shaped GeoFence with minimum radius and buffer <p>
   * When looking along the line from point A to B, right is positive distance, left is negative distance
   * @param pointA The first point
   * @param pointB The second point
   */
  public Line(Translation2d pointA, Translation2d pointB)
  {
    this(pointA, pointB, minRadius, minBuffer);
  }

  @Override
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    /*
    * Calculates the nearest point on the line to the robot
    * Uses the dot product of the lines A-B and A-Robot to project the robot position onto the line
    * Then clamps the calculated point between the line endpoints
    *      
    *            /              (robotX - aX) * (bX - aX) + (robotY - aY) * (bY - aY) \
    *      aXY + | (bXY - aXY) *   ------------------------------------------------   |
    *            \                            (bX - aX)^2 + (bY - aY)^2               /
    */

    double dot = (robotPos.getX() * dotX) + (robotPos.getY() * dotY) - dotXY; // Normalised dot product of the two lines
    return pointDamping
    (
      Conversions.clamp(pointA.getX() + dXab * dot, pointA.getX(), pointB.getX()), 
      Conversions.clamp(pointA.getY() + dYab * dot, pointA.getY(), pointB.getY()), 
      motionXY
    );
  }

  @Override
  public double getDistance(Translation2d testPos)
  {
    double dot = (testPos.getX() * dotX) + (testPos.getY() * dotY) - dotXY; // Normalised dot product of the two lines
    return new Translation2d
    (
      Conversions.clamp(pointA.getX() + dXab * dot, pointA.getX(), pointB.getX()), 
      Conversions.clamp(pointA.getY() + dYab * dot, pointA.getY(), pointB.getY())
    )
    .getDistance(testPos) - radius;
  }

  /** If the robot position is within the projection area of the line, the output will be negative on one side of the line */
  public double getDirectionalDistance()
    {return ((robotPos.getX() * normY) - (robotPos.getY() * normX) - normXY) - (radius + robotRadius);}

  /**
   * Constructs and adds one or more Attractors, relative to the line
   * @param antiNormal Reverse the approach direction between normal/antinormal to the line
   * @param normalOffset Distance away from the line along the approach direction, metres
   * @param tangentOffset Distance away from the line centre, metres right relative to the approach direction
   * @param effectRadius Distance at which the Attractor becomes active, metres
   * @param targetBuffer Distance at which the Robot must be moving along the approach direction, metres
   * @param activeCondition Condition for which the Attractor is active
   * @return The Line object with the new Attractor
   */
  public Line addRelativeAttractor(boolean antiNormal, double normalOffset, double tangentOffset, double effectRadius, double targetBuffer, BooleanSupplier activeCondition)
  {
    Translation2d unitNormal  = pointA.minus(pointB).div(length).rotateBy(antiNormal ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg);
    Translation2d unitTangent = unitNormal.rotateBy(Rotation2d.kCCW_90deg);
    Translation2d attractorCentre = centre.plus(unitNormal.times(normalOffset)).plus(unitTangent.times(tangentOffset));
    TriggerVector newAttractor = new TriggerVector
      (
        attractorCentre.getX(),
        attractorCentre.getY(),
        unitNormal.getAngle().getDegrees() + 180,
        effectRadius,
        targetBuffer
      );
    newAttractor.setActiveCondition(activeCondition);
    addAttractors(newAttractor);
    return this;
  }
}