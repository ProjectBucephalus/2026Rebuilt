package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;

/** 
 * Frequently-used maths operations <p>
 * Contains corrections or improvements of external implementations, specialised operations, and other utilities
 * @author 5985
 */
public class Conversions 
{
  /**
   * Mathematical modulus opperation, correcting the Java implimentation that incorrectly returns negative vaues
   * @param value input value
   * @param base base value of modulus
   * @return e.g. mod(8,10) == mod(18,10) == mod(-2,10) == 8
   */
  public static double mod(double value, double base)
  {
    value %= base;
    if (value < 0) {value += base;}
    return value;
  }
  
  /**
   * Modulus over the given range. Recursively adds or subtracts the size of the range until the result is within range
   * 
   * @param value number to wrap
   * @param min lowest end of target range, inclusive
   * @param max highest end of target range, inclusive
   * @return modulus of the number over the range
   */
  public static int wrap(int value, int min, int max)
  {
    if (max == min) return min;

    if (max < min)
      return wrapInner(value, max, min);
    else
      return wrapInner(value, min, max);
  }

  /** 
   * Inner function cannot handle bad inputs 
   * 
   * @param value number to wrap
   * @param min lowest end of target range, inclusive
   * @param max highest end of target range, inclusive
   * @return modulus of the number over the range
   */
  private static int wrapInner(int value, int min, int max)
  {
    if (value < min)
    {
      value += ((max-min) + 1);
      value = wrapInner(value, min, max);
    }
    else if (value > max)
    {
      value -= ((max-min) + 1);
      value = wrapInner(value,min,max);
    }

    return value;
  }

  /** 
   * MathUtil clamp, but allowing for limits to be in any order 
   * 
   * @param value input value to clamp
   * @param a maximum or minimum limit
   * @param b maximum or minimum limit
   * @return input value clamped between a and b
  */
  public static int clamp(int value, int a, int b)
    {return MathUtil.clamp(value, Math.min(a,b), Math.max(a,b));}
  
  /** 
   * MathUtil clamp, but allowing for limits to be in any order 
   * 
   * @param value input value to clamp
   * @param a maximum or minimum limit
   * @param b maximum or minimum limit
   * @return input value clamped between a and b
  */
  public static double clamp(double value, double a, double b)
    {return MathUtil.clamp(value, Math.min(a,b), Math.max(a,b));}

  /**
   * MathUtil clamp [-1..1] 
   * 
   * @param value input value to clamp
   * @return input value clamped between [-1..1]
  */
  public static double clamp(double value)
    {return MathUtil.clamp(value, -1, 1);}

  /**
   * Clamps the input Translation2d to a maximum length of 1 <p>
   * Does not modify the input if it's length is below 1
   * 
   * @param value input value to clamp the length of
   * @return input value scaled to have a length of at most 1
   */
  public static Translation2d clamp(Translation2d value)
  {
    double norm = value.getNorm();

    if (norm > 1)
      return value.div(norm);
    else 
      return value;
  }

  public static double round(double value, int places)
  {
    double factor = Math.pow(10, places);
    return Math.round(value * factor) / factor;
  }

  /**
   * Pose2d constructor wrapper to use raw doubles for all values <p>
   * NOTE: This is marginally inefficient for right-angles (0, 90, 180, etc), 
   * as there are pre-allocated constant Rotation2ds available
   * 
   * @param x x coordinate, meters
   * @param y y coordinate, meters
   * @param rotation heading, degrees
   */
  public static Pose2d buildPose(double x, double y, double rotation)
    {return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));}

  /**
   * Checks if two poses are within {@link Constants.ControlConstants#lineupTolerance lineupTolerance} of each other translationally, 
   * and {@link Constants.ControlConstants#angleLineupTolerance angleLineupTolerance} of each other rotationally
   * 
   * @param robotPose the first pose
   * @param targetPose the second pose
   * @return true if the poses are within tolerance of each other
   */
  public static boolean atPose(Pose2d robotPose, Pose2d targetPose)
    {return nearPose(robotPose, targetPose, Constants.ControlConstants.lineupTolerance, Constants.ControlConstants.angleLineupTolerance);}
  
  /**
   * Checks if two poses are within provided tolerances of each other translationally and rotationally
   * 
   * @param robotPose the first pose
   * @param targetPose the second pose
   * @param distanceTolerance the translational tolerance, meters
   * @param angleTolerance the rotational tolerance, degrees
   * @return true if the poses are within tolerance of each other
   */
  public static boolean nearPose(Pose2d robotPose, Pose2d targetPose, double distanceTolerance, double angleTolerance)
  {
    return 
      nearTranslation(robotPose.getTranslation(), targetPose.getTranslation(), distanceTolerance) 
      && nearRotation(robotPose.getRotation(), targetPose.getRotation(), angleTolerance);  
  }

  /**
   * Checks if two translations are within {@link Constants.ControlConstants#lineupTolerance lineupTolerance} of each other 
   * 
   * @param robotPos the first translation
   * @param targetPos the second translation
   * @return true if the two translations are within tolerance of each other
   */
  public static boolean atTranslation(Translation2d robotPos, Translation2d targetPos)
    {return nearTranslation(robotPos, targetPos, Constants.ControlConstants.lineupTolerance);}

  /**
   * Checks if two translations are within a provided tolerance of each other
   * 
   * @param robotPos the first translation
   * @param targetPos the second translation
   * @param distanceTolerance the tolerance, meters
   * @return true if the translations are within tolerance of each other
   */
  public static boolean nearTranslation(Translation2d robotPos, Translation2d targetPos, double distanceTolerance)
    {return robotPos.getDistance(targetPos) < distanceTolerance;}

  /**
   * Checks if two rotations are within {@link Constants.ControlConstants#angleLineupTolerance angleLineupTolerance} of each other, wrapping the angles
   * 
   * @param robotPose the first rotation
   * @param targetPose the second rotation
   * @return true if the rotations are within tolerance of each other
   */
  public static boolean atRotation(Rotation2d robotTheta, Rotation2d targetTheta)
    {return nearRotation(robotTheta, targetTheta, Constants.ControlConstants.angleLineupTolerance);}

  /**
   * Checks if two rotations are within a provided tolerance of each other, wrapping the angles
   * 
   * @param robotPose the first rotation
   * @param targetPose the second rotation
   * @param degreesTolerance the tolerance, degrees
   * @return true if the rotations are within tolerance of each other
   */
  public static boolean nearRotation(Rotation2d rotationA, Rotation2d rotationB, double degreesTolerance)
    {return nearRotation(rotationA.getDegrees(), rotationB.getDegrees(), degreesTolerance);}

  /**
   * Checks if two angles are within a provided tolerance of each other, wrapping the angles
   * 
   * @param angleA the first angle, degrees
   * @param angleB the second angle, degrees
   * @param degreesTolerance the tolerance, degrees
   * @return true if the angles are within tolerance of each other
   */
  public static boolean nearRotation(double angleA, double angleB, double degreesTolerance)
  {
    double difference = Math.abs(mod(angleA, 360) - mod(angleB, 360));

    return
      difference < 0 + degreesTolerance
      || difference > 360 - degreesTolerance;
  }

  /**
   * Navigates between relative angles for rotating mechanisms with greater than one rotation of freedom. <p>
   * Favours the centre of the allowed range when the shorter path is greater than 135 degrees
   * 
   * @param newAngle target relative angle, degrees, internally wrapped to [0..360]
   * @param currentAngle current absolute angle, degrees
   * @param maxAngle maximum allowed angle from centre, degrees (e.g. 720)
   * @return closest wrapping of target relative angle to current absolute angle within allowed range, degrees (e.g. [-720..720])
   */
public static double normaliseAngle(double newAngle, double currentAngle, double maxAngle)
{
  // Ensure allowed range of motion is greater than a rotation
  if (maxAngle < 180) return clamp(newAngle, -maxAngle, maxAngle);

  // Wrap both input angles to be strictly relative within a rotation
  // Find the shortest distance between the relative angles, wrapped [-180..180]
  double offset = MathUtil.inputModulus(mod(newAngle, 360) - mod(currentAngle, 360), -180, 180);
  
  // Find target absolute angle as current absolute angle plus offset between relative angles
  double targetAngle = currentAngle + offset;
  
  // If the target absolute angle is outside the allowed range, bring it one rotation towards centre    
  // When the travel is almost half a rotation, take the longer path if it brings the mechanism closer to centre
  if (targetAngle > maxAngle || (offset > 135 && currentAngle > 45))
    return targetAngle - 360;
  else if (targetAngle < -maxAngle || (offset < -135 && currentAngle < -45))
    return targetAngle + 360; 
  else 
    return targetAngle;
}
}