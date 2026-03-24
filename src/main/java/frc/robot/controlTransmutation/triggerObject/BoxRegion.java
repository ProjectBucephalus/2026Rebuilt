package frc.robot.controlTransmutation.triggerObject;

import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/** 
 * A rectangle shaped {@link TriggerRegion}
 * @author 5985
 */
public class BoxRegion extends TriggerRegion 
{
  private double Xa;
  private double Ya;
  private double Xb;
  private double Yb;

  /**
   * Rectangle shaped region
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   * @param radius Extra radius of region around the box (produces a rounded rectangle shape)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public BoxRegion(double xA, double yA, double xB, double yB, double radius, double buffer)
  {
    super(new Translation2d((xA + xB)/2, (yA + yB)/2), radius, buffer);

    this.Xa = Math.min(xA, xB);
    this.Ya = Math.min(yA, yB);
    this.Xb = Math.max(xA, xB);
    this.Yb = Math.max(yA, yB);

    checkRadius = (Math.hypot(xB - xA, yB - yA)/2) + radius + buffer;
  }

  /**
   * Rectangle shaped region with minimum radius and buffer size
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   */
  public BoxRegion(double Xa, double Ya, double Xb, double Yb)
    {this(Xa, Ya, Xb, Yb, minRadius, minBuffer);}

  @Override
  public double getDistance(Translation2d testPos)
  {
    double distance = 0;

    if (testPos.getX() < Xa)
    {
      if (testPos.getY() < Ya)
        {distance = Math.hypot(Xa - testPos.getX(), Ya - testPos.getY());}
      else if (testPos.getY() > Yb)
        {distance = Math.hypot(Xa - testPos.getX(), testPos.getY() - Yb);}
      else 
        {distance = Xa - testPos.getX();}
    }
    else if (testPos.getX() > Xb)
    {
      if (testPos.getY() < Ya)
        {distance = Math.hypot(testPos.getX() - Xb, Ya - testPos.getY());}
      else if (testPos.getY() > Yb)
        {distance = Math.hypot(testPos.getX() - Xb, testPos.getY() - Yb);}
      else
        {distance = testPos.getX() - Xb;}
    }
    else
    {
      distance = Math.max
      (
        Math.max
        (
          Xa - testPos.getX(), 
          Ya - testPos.getY()
        ),
        Math.max
        (
          testPos.getX() - Xb, 
          testPos.getY() - Yb
        )
      );
    }

    return distance - radius;
  }
}
