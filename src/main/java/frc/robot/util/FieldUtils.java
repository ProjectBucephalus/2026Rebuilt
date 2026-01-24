package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.GeoFencing;

public class FieldUtils 
{
  public static boolean isRedAlliance() 
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static Translation2d getAllianceHubCentre() 
  {
    return isRedAlliance() ? FieldConstants.redHubCentre : FieldConstants.blueHubCentre;
  }

  public static final int getDriverLocation()
  {
    if (DriverStation.getLocation().isPresent())
    {
      return DriverStation.getLocation().getAsInt();
    }
    else
    {
      return 0;
    }
  }

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

  public static Pose2d rotatePose(Pose2d pose) 
  {
    // flip pose when red
    if (isRedAlliance()) 
    {
      // reflect the pose around center point, flip both the X and Y position and rotation
      return pose.rotateAround(FieldConstants.fieldCentre, Rotation2d.k180deg);
    }

    // Blue or we don't know; return the original pose
    return pose;
  }

  public static void activateAllianceFencing(boolean redAlliance) 
  {
    GeoFencing.fieldRedGeoFence.setActiveCondition(() -> redAlliance);
    GeoFencing.fieldBlueGeoFence.setActiveCondition(() -> !redAlliance);
  }
}