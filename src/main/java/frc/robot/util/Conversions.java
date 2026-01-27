package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;


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
   * Modulus over the given range. Recursively adds or subtracts the size of the range until the result is within range.
   * @param value number to wrap
   * @param min lowest end of target range, inclusive
   * @param max highest end of target range, inclusive
   * @return modulus of the number over the range
   */
  public static int wrap(int value, int min, int max)
  {
    if (max == min)
    {
      return 0;
    }

    if (max < min)
    {
      int trueMax = min;
      min = max;
      max = trueMax;
    }

    if (value < min)
    {
      value += ((max-min) + 1);
      value = wrap(value, min, max);
    }
    else if (value > max)
    {
      value -= ((max-min) + 1);
      value = wrap(value,min,max);
    }
    return  value;
  }

  /** 
   * MathUtil clamp, but allowing for limits to be in any order 
   * @param value Input value
   * @param a Maximum or minimum limit
   * @param b Maximum or minimum limit
   * @return Input value clamped between a and b
  */
  public static int clamp(int value, int a, int b)
    {return MathUtil.clamp(value, Math.min(a,b), Math.max(a,b));}
  
  /** 
   * MathUtil clamp, but allowing for limits to be in any order 
   * @param value Input value
   * @param a Maximum or minimum limit
   * @param b Maximum or minimum limit
   * @return Input value clamped between a and b
  */
  public static double clamp(double value, double a, double b)
    {return MathUtil.clamp(value, Math.min(a,b), Math.max(a,b));}

  /** MathUtil clamp [-1..1] */
  public static double clamp(double value)
    {return MathUtil.clamp(value, -1, 1);}

  /** Returns the input T2D with a maximum length of 1 */
  public static Translation2d clamp(Translation2d value)
  {
    if (value.getNorm() > 1)
    {return value.div(value.getNorm());}
    return value;
  }

  /**
   * Pose2d Constructor Wrapper. Note that this is marginally inefficient for right-angles (0, 90, 180, etc), 
   * as there are pre-allocated constant Rotation2ds available
   * @param x X coordinate
   * @param y Y coordinate
   * @param rotation Heading, in degrees
   */
  public static Pose2d buildPose(double x, double y, double rotation)
    {return new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(rotation)));}

  public static boolean atPose(Pose2d robotPose, Pose2d targetPose)
    {return nearPose(robotPose, targetPose, Constants.Control.lineupTolerance, Constants.Control.angleLineupTolerance);}
  
  public static boolean nearPose(Pose2d robotPose, Pose2d targetPose, double distanceTolerance, double angleTolerance)
  {
    return 
      nearTranslation(robotPose.getTranslation(), targetPose.getTranslation(), distanceTolerance) 
      && nearRotation(robotPose.getRotation(), targetPose.getRotation(), angleTolerance);  
  }

  public static boolean atTranslation(Translation2d robotPos, Translation2d targetPos)
    {return nearTranslation(robotPos, targetPos, Constants.Control.lineupTolerance);}

  public static boolean nearTranslation(Translation2d robotPos, Translation2d targetPos, double distanceTolerance)
    {return robotPos.getDistance(targetPos) < distanceTolerance;}

  public static boolean atRotation(Rotation2d robotTheta, Rotation2d targetTheta)
    {return nearRotation(robotTheta, targetTheta, Constants.Control.angleLineupTolerance);}

  /** Returns true if the wrapped input angles are within the given tollerance */
  public static boolean nearRotation(Rotation2d rotationA, Rotation2d rotationB, double degreesTolerance)
  {
    return nearRotation(rotationA.getDegrees(), rotationB.getDegrees(), degreesTolerance);
  }

  /** Returns true if the wrapped input angles are within the given tollerance */
  public static boolean nearRotation(double angleA, double angleB, double degreesTolerance)
  {
    double difference = Math.abs(mod(angleA, 360) - mod(angleB, 360));

    return
      difference < 0 + degreesTolerance
      || difference > 360 - degreesTolerance;
  }

  public static double normaliseAngle(double newAngle, double currentAngle, double maxAngle)
  {
    // this handles the definitions for the different doubles
    double newAngleWrapped = Conversions.mod(newAngle, 360);
    double currentAngleWrapped = Conversions.mod(currentAngle, 360);

    double offset = MathUtil.inputModulus(newAngleWrapped - currentAngleWrapped, -180, 180);

    double targetAngle = currentAngleWrapped + offset;

    if (targetAngle > maxAngle)
      {return targetAngle - 360;}
    else if (targetAngle < -maxAngle)
      {return targetAngle + 360;}
    else 
      {return targetAngle;}
  }
}