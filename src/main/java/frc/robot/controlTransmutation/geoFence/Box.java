package frc.robot.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/**
 * A rectangle shaped {@link GeoFence} </p>
 * A cardinal rectangular region defined by two corners
 * @author 5985
 */
public class Box extends GeoFence
{
  private double Xa;
  private double Ya;
  private double Xb;
  private double Yb;

  /**
   * Rectangle shaped GeoFence
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   * @param radius Extra radius of geofence zone around the box (produces a rounded rectangle shape)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public Box(double Xa, double Ya, double Xb, double Yb, double radius, double buffer)
  {
    this.Xa = Math.min(Xa, Xb);
    this.Ya = Math.min(Ya, Yb);
    this.Xb = Math.max(Xa, Xb);
    this.Yb = Math.max(Ya, Yb);

    this.radius = radius;
    this.buffer = buffer;

    centre = new Translation2d((Xa + Xb)/2, (Ya + Yb)/2);

    checkRadius = (Math.hypot(Xb - Xa, Yb - Ya)/2) + radius + buffer;
  }

  /**
   * Rectangle shaped GeoFence with minimum radius and buffer
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   */
  public Box(double Xa, double Ya, double Xb, double Yb)
  {
    this(Xa, Ya, Xb, Yb, minRadius, minBuffer);
  }

  @Override
  public double getDistance(Translation2d testPos)
  {
    double distance = 0;

    if (testPos.getX() < Xa)
    {
      if (testPos.getY() < Ya) // SW Corner
        {distance = Math.hypot(Xa - testPos.getX(), Ya - testPos.getY());}
      else if (testPos.getY() > Yb) // NW Corner
        {distance = Math.hypot(Xa - testPos.getX(), testPos.getY() - Yb);}
      else // W Cardinal
        {distance = Xa - testPos.getX();}
    }
    else if (testPos.getX() > Xb)
    {
      if (testPos.getY() < Ya) // SE Corner
        {distance = Math.hypot(testPos.getX() - Xb, Ya - testPos.getY());}
      else if (testPos.getY() > Yb) // NE Corner
        {distance = Math.hypot(testPos.getX() - Xb, testPos.getY() - Yb);}
      else // E Cardinal
        {distance = testPos.getX() - Xb;}
    }
    else
    {
      // Inside, S Cardinal, or N Cardinal
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

    // Account for radius
    return distance - radius;
  }

  @Override
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    double motionX = motionXY.getX();
    double motionY = motionXY.getY();
    double distanceToEdge;

    if (robotPos.getX() < Xa)
    {
      if (robotPos.getY() < Ya) // SW Corner
        {return pointDamping(Xa, Ya, motionXY);}
      else if (robotPos.getY() > Yb) // NW Corner
        {return pointDamping(Xa, Yb, motionXY);}
      else // W Cardinal
      {
        distanceToEdge = (Xa - radius) - (robotPos.getX() + robotRadius);
        motionX = Math.min(motionX, (Conversions.clamp(distanceToEdge, 0, buffer)) / buffer);
      }
    }
    else if (robotPos.getX() > Xb)
    {
      if (robotPos.getY() < Ya) // SE Corner
        {return pointDamping(Xb, Ya, motionXY);}
      else if (robotPos.getY() > Yb) // NE Corner
        {return pointDamping(Xb, Yb, motionXY);}
      else // E Cardinal
      {
        distanceToEdge = (robotPos.getX() - robotRadius) - (Xb + radius);
        motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdge, 0, buffer)) / buffer);
      }
    }
    else 
    {
      if (robotPos.getY() < Ya) // S Cardinal
      {
        distanceToEdge = (Ya - radius) - (robotPos.getY() + robotRadius);
        motionY = Math.min(motionY, (Conversions.clamp(distanceToEdge, 0, buffer)) / buffer);
      } 
      else if (robotPos.getY() > Yb) // N Cardinal
      {
        distanceToEdge = (robotPos.getY() - robotRadius) - (Yb + radius);
        motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdge, 0, buffer)) / buffer);
      }
      else // Center (you've met a terrible fate *insert kazoo music here*)
        {return pointDamping(centre.getX(), centre.getY(), motionXY);}
    }

    // Check for collision
    if (distanceToEdge <= 0.05)
    {
      touchingObject = true; 
      PBDash.addToFieldObject
      (
        "Blocking Object", 
        Conversions.buildPose(Xa, Ya, 0),
        Conversions.buildPose(Xb, Ya, 0),
        Conversions.buildPose(Xb, Yb, 0),
        Conversions.buildPose(Xa, Yb, 0),
        Conversions.buildPose(Xa, Ya, 0),
        Conversions.buildPose(Xa - radius, Ya - radius, 0),
        Conversions.buildPose(Xb + radius, Ya - radius, 0),
        Conversions.buildPose(Xb + radius, Yb + radius, 0),
        Conversions.buildPose(Xa - radius, Yb + radius, 0),
        Conversions.buildPose(Xa - radius, Ya - radius, 0)
      );
    } 
    return new Translation2d(motionX, motionY);
  }
}