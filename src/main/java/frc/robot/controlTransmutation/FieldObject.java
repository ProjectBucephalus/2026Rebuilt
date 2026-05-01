package frc.robot.controlTransmutation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;

/**
 * Virtual objects on the field for changing inputs based on robot position
 * @author 5985
 */
public abstract class FieldObject implements InputTransmuter
{
  /** Global supplier of robot position */
  protected static Supplier<Translation2d> robotPosSup;
  /** Global cached value of robot position */
  protected static Translation2d robotPos;

  /** Global supplier of effective robot radius */
  protected static DoubleSupplier robotRadiusSup;
  /** Global cached value of effective robot radius */
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
  /** Condition for ALL objects to be active, if the return is false the object will return the input */
  protected static BooleanSupplier globalActiveSupplier = PBDash.IO_FENCE::get;

  /**
   * Sets the global robot position supplier for all field objects
   * @param robotPosSupplier Translation2d Supplier for the robot position (not pose)
   */
  public static void setRobotPosSup(Supplier<Translation2d> robotPosSupplier)
  {
    robotPosSup = robotPosSupplier;
    fetchRobotValues();
  }

  /**
   * Sets the global robot radius supplier for all field objects
   * @param robotRadiusSupplier Translation2d Supplier for the effective robot radius
   */
  public static void setRobotRadiusSup(DoubleSupplier robotRadiusSupplier)
    {robotRadiusSup = robotRadiusSupplier;}
  
  /** Pulls the robot radius and position from the suppliers into the global values for all field objects */
  public static void fetchRobotValues()
  {
    robotPos = robotPosSup.get();
    robotRadius = robotRadiusSup.getAsDouble();
  }

  /**
   * Calculates the distance between the robot and the field object
   * @return Distance to object, metres
   */
  public double getDistance()
  {
    return getDistance(robotPos) - robotRadius;
  }

  /**
   * Calculates the distance between the input point and the field object <p/>
   * Note: does not account for robot radius
   * @param testPos Position of robot to test, field coordinates
   * @return Distance to object, metres
   */
  public double getDistance(Translation2d testPos)
  {
    return centre.getDistance(testPos) - radius;
  }

  /**
   * Returns the centrepoint of the field object
   * @return XY of the centre of the object, metres
   */
  public Translation2d getCentre()
    {return centre;}

  /**
   * Runs minimum necessary checks on the robot position before running more intense processing
   * @return True if further processing is required
   */
  protected boolean checkPosition()
    {return centre.getDistance(robotPos) <= (checkRadius + robotRadius);}

  /**
   * Tests if the given point is touching or inside the active object
   * @param testPos Position of robot to test, field coordinates
   * @return {@code true} if the object is active and the distance to point is <= 0
   */
  public boolean checkPosition(Translation2d testPos)
  {
    return activeSupplier.getAsBoolean() && getDistance(testPos) <= 0;
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

  /**
   * Sets the condition for which ALL field objects are active
   * @param newActiveCondition Any BooleanSupplier, if true field objects will be processed
   */
  public static void setGlobalActiveCondition(BooleanSupplier newActiveCondition)
    {globalActiveSupplier = newActiveCondition;}
}
