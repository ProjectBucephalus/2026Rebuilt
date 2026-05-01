package frc.robot.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/**
 * A wall shaped {@link GeoFence} </p>
 * The outer wall that the robot must stay within <p>
 * A cardinal rectangular region defined by two corners
 * @author 5985
 */
public class Fence extends GeoFence
{
  private double Xa;
  private double Ya;
  private double Xb;
  private double Yb;

  /**
   * Wall shaped GeoFence
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   * @param radius Extra radius of geofence zone on the inner walls of the fence
   * @param buffer Buffer within the walls over which the speed is reduced
   */
  public Fence(double Xa, double Ya, double Xb, double Yb, double radius, double buffer)
  {
    this.Xa = Math.min(Xa, Xb);
    this.Ya = Math.min(Ya, Yb);
    this.Xb = Math.max(Xa, Xb);
    this.Yb = Math.max(Ya, Yb);

    this.radius = radius;
    this.buffer = buffer;

    centre = new Translation2d((Xa + Xb)/2, (Ya + Yb)/2);

    checkRadius = radius + buffer;
  }

  /**
   * Wall shaped GeoFence with minimum radius and buffer
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   */
  public Fence(double Xa, double Ya, double Xb, double Yb)
  {
    this(Xa, Ya, Xb, Yb, minRadius, minBuffer);
  }

  @Override
  public double getDistance(Translation2d testPos)
  {
    return Math.abs
    (
      Math.min
      (
        Math.min(testPos.getX() - (Xa + radius), (Xb - radius) - testPos.getX()),
        Math.min((Yb - radius) - testPos.getY(), testPos.getY() - (Ya + radius))
      )
    );
  }

  @Override
  protected boolean checkPosition()
  {
    return
    (
      (robotPos.getX() >= Xb - (checkRadius + robotRadius)) ||  // Close to inside of +X barrier
      (robotPos.getX() <= Xa + (checkRadius + robotRadius)) ||  // Close to inside of -X barrier
      (robotPos.getY() >= Yb - (checkRadius + robotRadius)) ||  // Close to inside of +Y barrier
      (robotPos.getY() <= Ya + (checkRadius + robotRadius))     // Close to inside of -Y barrier
    );
  }

  @Override
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    // Calculates distance to the relevant edge of the field
    // Calculates edge position, and subtracts robot position + radius from edge position.

    // Sets the motion in the relevant direction to the minimum of the current motion
    // And the distance from the edge clamped between 0 and the edge buffer, and normalised to a maximum of 1.
    // This ensures the motion in that direction does not go above the clamped + normalised distance from the edge, to cap speed.
    
    double motionX = motionXY.getX();
    double motionY = motionXY.getY();

    // Holding values initialised as non-zero to allow for collision check
    double distanceToEdgeX = 1;
    double distanceToEdgeY = 1;
    
    if (motionX > 0)
    {   
      distanceToEdgeX = (Xb - radius) - (robotPos.getX() + robotRadius); 
      motionX = Math.min(motionX, (Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
    }
    else if (motionX < 0)
    {   
      distanceToEdgeX = (robotPos.getX() - robotRadius) - (Xa + radius);
      motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
    }

    if (motionY > 0)
    {   
      distanceToEdgeY = (Yb - radius) - (robotPos.getY() + robotRadius);
      motionY = Math.min(motionY, (Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
    }
    else if (motionY < 0)
    {   
      distanceToEdgeY = (robotPos.getY() - robotRadius) - (Ya + radius);
      motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
    }

    // Check for collision
    if(distanceToEdgeX <= 0.05 || distanceToEdgeY <= 0.05)
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
        Conversions.buildPose(Xa + radius, Ya + radius, 0),
        Conversions.buildPose(Xb - radius, Ya + radius, 0),
        Conversions.buildPose(Xb - radius, Yb - radius, 0),
        Conversions.buildPose(Xa + radius, Yb - radius, 0),
        Conversions.buildPose(Xa + radius, Ya + radius, 0)
      );
    } 

    return new Translation2d(motionX, motionY);
  }
}