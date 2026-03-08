package frc.robot.util.controlTransmutation.restrictor;

import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/** 
 * A rectangle shaped {@link Restrictor}
 * @author 5985
 */
public class BoxRestrictor extends Restrictor 
{
  private double Xa;
  private double Ya;
  private double Xb;
  private double Yb;

  /**
   * Rectangle shaped restrictor
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   * @param radius Extra radius of restrictor zone around the box (produces a rounded rectangle shape)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public BoxRestrictor(double xA, double yA, double xB, double yB, double radius, double buffer)
  {
    super(new Translation2d((xA + xB)/2, (yA + yB)/2), radius, buffer);

    this.Xa = Math.min(xA, xB);
    this.Ya = Math.min(yA, yB);
    this.Xb = Math.max(xA, xB);
    this.Yb = Math.max(yA, yB);

    checkRadius = (Math.hypot(xB - xA, yB - yA)/2) + radius + buffer;
  }

  /**
   * Rectangle shaped restrictor with minimum radius and buffer size
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   */
  public BoxRestrictor(double Xa, double Ya, double Xb, double Yb)
    {this(Xa, Ya, Xb, Yb, minRadius, minBuffer);}

  @Override
  public double getDistance()
  {
    double distance = 0;

    if (robotPos.getX() < Xa)
    {
      if (robotPos.getY() < Ya)
        {distance = Math.hypot(Xa - robotPos.getX(), Ya - robotPos.getY());}
      else if (robotPos.getY() > Yb)
        {distance = Math.hypot(Xa - robotPos.getX(), robotPos.getY() - Yb);}
      else 
        {distance = Xa - robotPos.getX();}
    }
    else if (robotPos.getX() > Xb)
    {
      if (robotPos.getY() < Ya)
        {distance = Math.hypot(robotPos.getX() - Xb, Ya - robotPos.getY());}
      else if (robotPos.getY() > Yb)
        {distance = Math.hypot(robotPos.getX() - Xb, robotPos.getY() - Yb);}
      else
        {distance = robotPos.getX() - Xb;}
    }
    else
    {
      distance = Math.max
      (
        Math.max
        (
          Xa - robotPos.getX(), 
          Ya - robotPos.getY()
        ),
        Math.max
        (
          robotPos.getX() - Xb, 
          robotPos.getY() - Yb
        )
      );
    }

    return distance - (radius + robotRadius);
  }
}
