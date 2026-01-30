package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.GeoFencing;

/** Field or FMS related utilities */
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
    {return isRedAlliance() ? FieldConstants.redHubCentre : FieldConstants.blueHubCentre;}

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
  public static Pose2d flipPose(Pose2d pose) 
  {
    // flip pose when red
    if (isRedAlliance()) 
    {
      Rotation2d rot = pose.getRotation();
      // reflect the pose over center line, flip both the X and the rotation
      return new Pose2d(FieldConstants.fieldLength - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
    }

    // Blue or we don't know; return the original pose
    return pose;
  }

  /**
   * Rotates the provided pose if we're on the red alliance
   * 
   * @param pose a blue-origin pose
   * @return the pose rotated to match our alliance
   */
  public static Pose2d rotatePose(Pose2d pose) 
  {
    // flip pose when red
    if (isRedAlliance()) 
      // reflect the pose around center point, flip both the X and Y position and rotation
      return pose.rotateAround(FieldConstants.fieldCentre, Rotation2d.k180deg);
    else 
      // Blue or we don't know; return the original pose
      return pose;
  }

  /**
   * Activates the relevant geofences for our alliance
   * 
   * @param redAlliance whether we're on the red alliance
   */
  public static void activateAllianceFencing() 
  {
    boolean redAlliance = isRedAlliance();
    GeoFencing.fieldRedGeoFence.setActiveCondition(() -> redAlliance);
    GeoFencing.fieldBlueGeoFence.setActiveCondition(() -> !redAlliance);
  }
}