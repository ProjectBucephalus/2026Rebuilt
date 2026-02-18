package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.robot.constants.FieldConstants.*;
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

  /**
   * Checks if the provided position is within our alliance zone
   * 
   * @param pos Position to check against
   * @return If the position is within the alliance zone
   */
  public static boolean inLeftHalf(Translation2d pos) 
  {
    return isRedAlliance() 
      ? pos.getY() < fieldCentre.getY()
      : pos.getY() > fieldCentre.getY();
  }
}