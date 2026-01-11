package frc.robot.util;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.util.controlTransmutation.geoFence.Line;

public class FieldUtils 
{
  public static boolean isRedAlliance() 
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
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

  public static int getNearestReefFaceAllianceLocked(Translation2d robotPos)
  {
    int nearestReefFace;
    ArrayList<Translation2d> localList = FieldConstants.blueReefMidpoints;

    nearestReefFace = localList.indexOf(robotPos.nearest(localList)); 

    nearestReefFace = (int)Conversions.wrap(nearestReefFace, 1, 6);
    
    return nearestReefFace;
  }

  public static int getNearestReefFace(Translation2d robotPos)
  {
    int nearestReefFace;
    ArrayList<Translation2d> localList =
      isRedAlliance() ? 
      FieldConstants.redReefMidpoints :
      FieldConstants.blueReefMidpoints;

    nearestReefFace = localList.indexOf(robotPos.nearest(localList)); 

    nearestReefFace = (int)Conversions.wrap(nearestReefFace, 1, 6);
    
    return nearestReefFace;
  }

  public static Line getNearestCoralStation(Translation2d robotPos)
  {
    boolean northHalf = robotPos.getY() >= FieldConstants.fieldCentre.getY();

    return
    robotPos.getX() >= FieldConstants.fieldCentre.getX() ?
    northHalf ? GeoFencing.cornerNRed : GeoFencing.cornerSRed
    :
    northHalf ? GeoFencing.cornerNBlue : GeoFencing.cornerSBlue;
  }

  public static boolean atReefLineUp(Pose2d robotPose)
    {return Arrays.stream(FieldConstants.reefLineups).anyMatch(lineup -> atPose(robotPose, lineup));}

  public static boolean atPose(Pose2d robotPose, Pose2d targetPose)
  {
    return robotPose.getTranslation().getDistance(targetPose.getTranslation()) < Constants.Control.lineupTolerance &&
    Math.abs(robotPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < Constants.Control.angleLineupTolerance;   
  }
  
  public static boolean nearPose(Pose2d robotPose, Pose2d targetPose, double distanceTolerance)
  {
    return robotPose.getTranslation().getDistance(targetPose.getTranslation()) < (Constants.Control.lineupTolerance + distanceTolerance) &&
    Math.abs(robotPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < Constants.Control.angleLineupTolerance;   
  }
}