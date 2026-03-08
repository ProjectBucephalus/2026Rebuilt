package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.robot.constants.FieldConstants.*;

import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.FieldConstants.GeoFencing;
import static frc.robot.constants.Constants.SwerveConstants.robotRadiusInscribed;

/** 
 * Field or FMS related utilities 
 * @author 5985
 */
public class FieldUtils 
{
  private static boolean redAlliance;
  static {updateAlliance();}

  // timer function variables
  private static double autoStart = 0;            //variable to save system time at start of auto
  private static double teleStart = 0;            //variable to save system time at start of teleop
  private static double MAX_GAME_TIME = 165;   //match length in millis
  private static double AUTO_TIME = 15;        //Auto length in millis
  private static double TELE_TIME = 150;       //Teleop length in millis

  /**
   * Checks whether we are on the red alliance <p>
   * If the alliance value is unavailable for some reason, it will always return false (i.e. blue alliance)
   * 
   * @return true if we are on the red alliance
   */
  public static boolean isRedAlliance() 
    {return redAlliance;}

  /**
   * Updates the cached alliance value 
   */
  public static void updateAlliance()
    {redAlliance = DriverStation.getAlliance().map(Alliance.Red::equals).orElse(false);}

  /** 
   * @return the centre point of your alliance's hub
   */
  public static Translation2d getAllianceHubCentre() 
    {return isRedAlliance() ? redHubCentre : blueHubCentre;}

  /**
   * @return which driver station we are being controlled from (1, 2, or 3), or 0 if the value is unavailable
   */
  public static final int getDriverLocation()
    {return DriverStation.getLocation().orElse(0);}

  /**
   * Mirrors the provided pose if we're on the red alliance
   * 
   * @param pose a blue-origin pose
   * @return the pose mirrored to match our alliance
   */
  public static Pose2d allianceFlipPose(Pose2d pose) 
  {
    // flip pose when red
    if (isRedAlliance()) 
    {
      return flipPose(pose);
    }

    // Blue or we don't know; return the original pose
    return pose;
  }

  /**
   * Mirrors the provided pose
   * 
   * @param pose original pose
   * @return the pose mirrored accross field centreline
   */
  public static Pose2d flipPose(Pose2d pose) 
  {
    Rotation2d rot = pose.getRotation();
    // reflect the pose over center line, flip both the X and the rotation
    return new Pose2d(fieldLength - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
  }

  /**
   * Rotates the provided pose if we're on the red alliance
   * 
   * @param pose a blue-origin pose
   * @return the pose rotated to match our alliance
   */
  public static Pose2d allianceRotatePose(Pose2d pose) 
  {
    // flip pose when red
    if (isRedAlliance()) 
      // reflect the pose around center point, flip both the X and Y position and rotation
      return pose.rotateAround(fieldCentre, Rotation2d.k180deg);
    else 
      // Blue or we don't know; return the original pose
      return pose;
  }

  /**
   * Rotates the provided pose
   * 
   * @param pose original pose
   * @return the pose rotated around the field centre
   */
  public static Pose2d rotatePose(Pose2d pose) 
  {
    // reflect the pose around center point, flip both the X and Y position and rotation
    return pose.rotateAround(fieldCentre, Rotation2d.k180deg);
  }

  /**
   * Mirrors the provided translation accross the field centreline if we're on the red alliance
   * 
   * @param translation a blue-origin translation
   * @return the translation mirrored to match our alliance
   */
  public static Translation2d allianceFlipTranslation(Translation2d translation) 
  {
    // flip translation when red
    if (isRedAlliance()) 
      // reflect the translation around center point, flip both the X and Y position
      return flipTranslation(translation);
    else 
      // Blue or we don't know; return the original translation
      return translation;
  }

  /**
   * Mirrors the provided translation accross the field centreline
   * 
   * @param translation original translation
   * @return the translation mirrored accross field centre
   */
  public static Translation2d flipTranslation(Translation2d translation) 
  {
    // reflect the translation around center point, flip both the X and Y position
    return translation.rotateAround(fieldCentre, Rotation2d.k180deg);
  }

  /**
   * Rotates the provided translation if we're on the red alliance
   * 
   * @param translation a blue-origin translation
   * @return the translation rotated to match our alliance
   */
  public static Translation2d allianceRotateTranslation(Translation2d translation) 
  {
    // flip translation when red
    if (isRedAlliance()) 
      // reflect the translation around center point, flip both the X and Y position
      return rotateTranslation(translation);
    else 
      // Blue or we don't know; return the original translation
      return translation;
  }

  /**
   * Rotates the provided translation
   * 
   * @param translation original translation
   * @return the translation rotated around field centre
   */
  public static Translation2d rotateTranslation(Translation2d translation) 
  {
    // reflect the translation around center point, flip both the X and Y position
    return translation.rotateAround(fieldCentre, Rotation2d.k180deg);
  }

  /**
   * Activates the relevant geofences for our alliance
   * 
   * @param redAlliance whether we're on the red alliance
   */
  public static void activateAllianceFencing() 
  {
    GeoFencing.fieldRedGeoFence.setActiveCondition(() -> isRedAlliance());
    GeoFencing.fieldBlueGeoFence.setActiveCondition(() -> !isRedAlliance());
  }

  /**
   * Checks if the provided position is within our alliance zone
   * 
   * @param pos Position to check against
   * @return If the position is within the alliance zone
   */
  public static boolean inAllianceZone(Translation2d pos) 
  {
    if (isRedAlliance()) 
      return pos.getX() > redStartLine.getX() - robotRadiusInscribed;
    else 
      return pos.getX() < blueStartLine.getX() + robotRadiusInscribed;
  }

  public static Translation2d getClosestPassPoint(Translation2d pos)
  {
    boolean inLeftHalf = isRedAlliance() 
      ? pos.getY() < fieldCentre.getY()
      : pos.getY() > fieldCentre.getY();

    return inLeftHalf ? ControlConstants.leftFerryTarget.get() : ControlConstants.rightFerryTarget.get();
  }


  /**
   * Timer Functions
   */
  
   /**
    * internal wrapper for system time to return in seconds instead of millis 
    * @return current system time in seconds.
    */
  public static double currentTime()
  {
    return (double)System.currentTimeMillis()/1000;
  }

  /**
   * Called at the start of auto to mark the beginning of the match
   *  
   */ 
  public static void startAuto()
  {
    autoStart = currentTime();
  }

  /**
   * Called at the start of teleop to mark the beginning of the teleop period
   * 
   */
  public static void startTele()
  {
    teleStart = currentTime();
  }

  /**
   * Gets game time elapsed in seconds.
   * 
   * @return the number of seconds since the startAuto() call,
   * returns zero if startAuto() has not been called or the match is over.
   */
  public static double getGameTimeElapsed()
  {
    double timeNow = currentTime();
    if ((timeNow < autoStart) || (timeNow > (autoStart + MAX_GAME_TIME)))
    {
      return 0;
    }
    else
    {
      return timeNow-autoStart;
    }
  }

  /**
   * Gets the number of seconds remaining in the current match.
   * 
   * @return the time remaining in the current match, in seconds.
   * Returns zero if startAuto has not been called or the match is over.
   */
  public static double getGameTimeRemaining()
  {
    double timeNow = currentTime();
    if ((timeNow < autoStart) || (timeNow > (autoStart + MAX_GAME_TIME)))
    {
      return 0;
    }
    else
    {
      return MAX_GAME_TIME - (timeNow - autoStart);
    }
  }

  /**
   * Gets the time elapsed in the current autonomous period.
   * NB while in auto should be identical to getGameTimeElapsed()
   * 
   * @return time elapsed since the startAuto() call, in seconds.
   * Will return zero if startAuto() has not been called, and 15 if auto is finished.
   */
  public static double getAutoTimeElapsed()
  {
    double timeNow = currentTime();
    if (timeNow < autoStart)
    {
      return 0;
    }
    else
    {
      return Math.min(timeNow - autoStart, AUTO_TIME);
    }
  }

  /**
   * Gets the time remaining in the current autonomous period.
   * 
   * @return the time remaining in the current auto, in seconds.
   * Will return zero if startAuto() has not been called, or if auto is finished.
   */
  public static double getAutoTimeRemaining()
  {
    double timeNow = currentTime();
    if ((timeNow < autoStart) || (timeNow > autoStart + AUTO_TIME))
    {
      return 0;
    }
    else
    {
      return AUTO_TIME - (timeNow - autoStart);
    }
  }

  /**
   * Gets the time elapsed during the current teleoperated period.
   * 
   * @return the time elapsed since the startTele() call, in seconds.
   * Will return zero if startTele() has not been called, or the match is over.
   */
  public static double getTeleTimeElapsed()
  {
    double timeNow = currentTime();
    if ((timeNow < teleStart) || (timeNow > (teleStart + TELE_TIME)))
    {
      return 0;
    }
    else
    {
      return timeNow - teleStart;
    }
  }

  /**
   * Gets the time remaining in the current teleoperated period.
   * 
   * @return the time remaining in the current teleop, in seconds.
   * Will return zero if startTele() has not been called, or the match is over.
   */
  public static double getTeleTimeRemaining()
  {
    double timeNow = currentTime();
    if ((timeNow < teleStart) || (timeNow > (teleStart + TELE_TIME)))
    {
      return 0;
    }
    else
    {
      return TELE_TIME - (timeNow - teleStart);
    }
  }
  
}