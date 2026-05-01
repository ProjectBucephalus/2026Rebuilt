package frc.robot.controlTransmutation.geoFence;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.controlTransmutation.FieldObject;
import frc.robot.controlTransmutation.triggerObject.TriggerVector;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;

/** 
 * Derived from video-game collision-detection, GeoFence objects combine field-relative driving and localisation
 * to make the robot respect virtual barriers as physical, smoothly reducing and shifting inputs
 * to stop at the defined edge, or glide around corners.
 * @author 5985
 */
public abstract class GeoFence extends FieldObject
{
  // Inherits from FieldObject: T2D centre, double radius, double buffer, double checkRadius

  /** Flag to indicate per cycle if the robot is in contact with an active GeoFence object */
  protected static boolean touchingObject;

  // A list of object-relative attractors to check
  protected ArrayList<TriggerVector> attractors = new ArrayList<>();

  /** @return {@code true} if the minimum distance between the robot and an active GeoFence object is <= 0.05m */
  public static boolean isBlocked()
    {return touchingObject;}

  /**
   * Clears the flag indicating that the robot is in contact with a GeoFence object
   * @return the state of the flag before clearing
   */
  public static boolean clearBlocked()
    {
      boolean wasTouching = touchingObject;
      touchingObject = false;
      if (!wasTouching) PBDash.putFieldObject("Blocking Object");
      return wasTouching;
    }

  /**
   * Adds one or more Attractor objects tied to the GeoFence object
   * @param newAttractors list of Attractors in Field coordinates
   * @return the GeoFence object with the new Attractors
   */
  public GeoFence addAttractors(TriggerVector ...newAttractors)
  {
    attractors.addAll(List.of(newAttractors));
    return this;
  }

  /** Applies any contained attractors and then applies the geofencing */
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    if (globalActiveSupplier.getAsBoolean() && activeSupplier.getAsBoolean())
    {
      // if (checkAttractors())
      // {
      //   var controlOutput = processAttractors(controlInput);
      //   // If the attractors have not had any affect, we want to continue to the geofence checks rather than returning
      //   if (!controlOutput.equals(controlInput))
      //     return controlOutput;
      // }

      if (checkPosition())
        return dampMotion(controlInput);
    }
    
    return controlInput;
  }

  /**
   * Checks the list of attractors to see if any need further processing
   * @return True if any attached attractors need further processing
   */
  public boolean checkAttractors()
  {
    for (var attractor : attractors)
      if (attractor.checkPosition())
        return true;

    return false;
  }

  /**
   * Sorts the attractors by distance and returns the output of the closest valid one
   * @param controlInput Original joystick input, [-1..1],[-1..1]
   * @return Processed joystick output, [-1..1],[-1..1]
   */
  public Translation2d processAttractors(Translation2d controlInput)
  {
    return attractors
      .stream()
      .min(Comparator.comparingDouble(line -> line.getCentre().getDistance(robotPos)))
      .orElseThrow()
      .process(controlInput);
  }
  
  /**
   * Modifies the input to prevent the robot from entering the object, while allowing a robot that is inside of it to escape
   * @param motionXY XY control input, field-relative, [-1..1],[-1..1]
   * @return XY control output, field-relative, [-1..1],[-1..1]
   */
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    return motionXY;
  }

  /**
   * Damps the input motion relative to the given point, such that the normal component is zero when touching the object
   * @param pointX X-coordinate of the point
   * @param pointY Y-coordinate of the point
   * @param motionXY XY control input to be processed
   * @return Control output with the normal compoenent damped
   */
  protected Translation2d pointDamping(double pointX, double pointY, Translation2d motionXY)
  {
    // Calculates X and Y distances to the point
    double distanceX = pointX - robotPos.getX();
    double distanceY = pointY - robotPos.getY();
    // Calculates the normal distance to the corner through pythagoras; this is the actual distance between the robot and point
    double distanceN = Math.hypot(distanceX, distanceY);
    // Calculates the robot's motion normal and tangent to the point; i.e., towards and away from the point, and from side to side relative to the point
    double motionN   = ((distanceX * motionXY.getX()) + (distanceY * motionXY.getY())) / distanceN;
    double motionT   = ((distanceX * motionXY.getY()) - (distanceY * motionXY.getX())) / distanceN;
    
    // Clamps the normal motion, i.e. motion towards the point, in order to clamp robot speed
    // Sets maximum input towards the object as:
    //      (position within the buffer normalised to [0..1])   *   (angle normalisation factor [1..sqrt(2)])
    //         (dNormal - object radii)[0..buffer] / buffer     *      (mNormal / max(|X|,|Y|))
    motionN = Math.min(motionN, motionN * Conversions.clamp(distanceN-(robotRadius + radius), 0, buffer)
                                    / (Math.max(Math.abs(distanceX),Math.abs(distanceY)) * buffer));
    
    // Converts clamped motion from normal back to X and Y
    double motionX   = ((motionN * distanceX) - (motionT * distanceY)) / distanceN;
    double motionY   = ((motionN * distanceY) + (motionT * distanceX)) / distanceN;

    // Check for collision
    if (distanceN <= 0.05 + robotRadius + radius) 
    {
      touchingObject = true; 
      PBDash.addToFieldObject
      (
        "Blocking Object", 
        Conversions.buildPose(pointX, pointY, 0),
        Conversions.buildPose(pointX + radius, pointY, 0),
        Conversions.buildPose(pointX, pointY + radius, 0),
        Conversions.buildPose(pointX - radius, pointY, 0),
        Conversions.buildPose(pointX, pointY - radius, 0),
        Conversions.buildPose(pointX, pointY + radius, 0),
        Conversions.buildPose(pointX + radius, pointY, 0),
        Conversions.buildPose(pointX - radius, pointY, 0)
      );
    }
    return new Translation2d(motionX, motionY);
  }
}
