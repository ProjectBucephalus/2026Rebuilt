package frc.robot.util.controlTransmutation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Virtual objects on the field for changing inputs based on robot position
 */
public abstract class FieldObject extends InputTransmuter
{
  /** Global supplier of robot position */
  protected static Supplier<Translation2d> robotPosSup;
  /** Global value of robot position */
  protected static Translation2d robotPos;
  /** Global supplier of effective robot radius */
  protected static DoubleSupplier robotRadiusSup;
  /** Global value of effective robot radius */
  protected static double robotRadius;
  /** Object centrepoint, metres */
  protected Translation2d centre;
  /** Radius of the object from the centre/lines, metres */
  protected double radius;
  /** Range over which the effect of the object transitions from 0 to Max, metres */
  protected double buffer;
  /** Distance at which further processing is required, metres */
  protected double checkRadius;
  /** Condition for the object to be active, if the return is false the object will return the input */
  protected BooleanSupplier activeSupplier = () -> true;

  /**
   * Sets the global robot position supplier for all field objects
   * @param robotPosSupplier Translation2d Supplier for the robot position (not pose)
   */
  public static void setRobotPosSup(Supplier<Translation2d> robotPosSupplier)
  {
    robotPosSup = robotPosSupplier;
  }
  
  /** Pulls the robot position from the supplier into the global value for all field objects */
  public static void fetchRobotPos()
  {
    robotPos = robotPosSup.get();
    robotRadius = robotRadiusSup.getAsDouble();
  }
  
  /**
   * Sets the global robot radius supplier for all field objects
   * @param robotRadiusSupplier Translation2d Supplier for the effective robot radius
   */
  public static void setRobotRadiusSup(DoubleSupplier robotRadiusSupplier)
  {
    robotRadiusSup = robotRadiusSupplier;
  }

  /**
   * Calculates the distance between the robot and the field object
   * @return Distance to object, metres
   */
  public double getDistance()
  {
    return centre.getDistance(robotPos) - (radius + robotRadius);
  }

  /**
   * Returns the centrepoint of the field object
   * @return XY of the centre of the object, metres
   */
  public Translation2d getCentre()
  {
    return centre;
  }

  /**
   * Runs minimum necessary checks on the robot position before running more intense processing
   * @return True if further processing is required
   */
  protected boolean checkPosition()
  {
    return centre.getDistance(robotPos) <= checkRadius + robotRadius;
  }

  /**
   * Sets the condition for which the object is active
   * @param newActiveCondition Any BooleanSupplier, if true the object will be processed
   * @return The FieldObject with the new active condition
   */
  public FieldObject setActiveCondition(BooleanSupplier newActiveCondition)
  {
    activeSupplier = newActiveCondition;
    return this;
  }
}
