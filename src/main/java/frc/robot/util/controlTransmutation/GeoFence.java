package frc.robot.util.controlTransmutation;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;

/** Add your docs here. */
public abstract class GeoFence extends FieldObject
{
  // Inherits from FieldObject: T2D centre, double radius, double buffer, double checkRadius
  protected ArrayList<Attractor> attractors = new ArrayList<Attractor>();

  /**
   * Adds one or more Attractor objects tied to the GeoFence object
   * @param newAttractors list of Attractors in Field coordinates
   * @return the GeoFence object with the new Attractors
   */
  public GeoFence addAttractors(Attractor ...newAttractors)
  {
    for (int i = 0; i < newAttractors.length; i++)
    {
      attractors.add(newAttractors[i]);
    }
    return this;
  }

  @Override
  public Translation2d process(Translation2d controlInput)
  {
    if (activeSupplier.getAsBoolean())
    {
      Translation2d controlOutput;
      if (checkAttractors())
      {
        controlOutput = processAttractors(controlInput);
        if (!controlOutput.equals(controlInput))
          {return controlOutput;}
      }
      if (checkPosition())
        {return dampMotion(controlInput);}
    }
    
    return controlInput;
  }

  /**
   * Checks the list of attractors to see if any need further processing
   * @return True if any attached attractors need further processing
   */
  public boolean checkAttractors()
  {
    if (attractors.size() > 0)
    {
      for (int i = 0; i < attractors.size(); i++)
      {
        if (attractors.get(i).checkPosition())
          {return true;}
      }
    }

    return false;
  }

  /**
   * Sorts the attractors by distance and returns the output of the closest valid one
   * @param controlInput Original joystick input, [-1..1],[-1..1]
   * @return Processed joystick output, [-1..1],[-1..1]
   */
  public Translation2d processAttractors(Translation2d controlInput)
  {
    double distance = 100;
    int index = 0;

    for (int i = 0; i < attractors.size(); i++)
      {
        if (attractors.get(i).checkAngle(controlInput) && attractors.get(i).getDistance() < distance)
        {
          distance = attractors.get(i).getDistance();
          index = i;
        }
      }

    return attractors.get(index).process(controlInput);
  }
  
  /**
   * Modifies the input to prevent the robot from entering the object
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
    return new Translation2d(motionX, motionY);
  }
}
