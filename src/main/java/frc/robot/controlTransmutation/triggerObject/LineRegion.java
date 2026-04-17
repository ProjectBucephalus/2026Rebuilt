package frc.robot.controlTransmutation.triggerObject;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;

/** 
 * A line shaped region <p>
 * Defined between two points </p>
 * Due to having no area, has no active zone. Used purely as a distance check
 * @author 5985
 */
public class LineRegion extends TriggerRegion
{
  private Translation2d pointA;
  private Translation2d pointB;    

  private double length;

  private double dXab;
  private double dYab;

  private double normX;
  private double normY;
  private double normXY;

  private double dotX;
  private double dotY;
  private double dotXY;

  /**
   * Line shaped region <p>
   * When looking along the line from point A to B, right is positive distance, left is negative distance
   * @param pointA The first point
   * @param pointB The second point
   */
  public LineRegion(Translation2d pointA, Translation2d pointB)
  {
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

  /**
   * Calculates the distance between the robot and the line, accounting for the robot being on either side of the line
   * @return Directional distance to the line, meters
   */
  public double getDirectionalDistance()
  {
    double dot = (robotPos.getX() * dotX) + (robotPos.getY() * dotY) - dotXY; // Normalised dot product of the two lines
    if (dot <= 0)
      {return pointA.getDistance(robotPos);}
    else if (dot >= 1)
      {return pointB.getDistance(robotPos);}
    else
      {return ((robotPos.getX() * normY) + (robotPos.getY() * normX) - normXY);}
  }
}