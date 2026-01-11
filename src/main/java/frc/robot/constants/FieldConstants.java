package frc.robot.constants;

import java.util.ArrayList;
import java.util.function.BiPredicate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot.DriveState;
import frc.robot.Robot.TargetPosition;
import frc.robot.util.controlTransmutation.Attractor;
import frc.robot.util.controlTransmutation.ObjectList;
import frc.robot.util.controlTransmutation.geoFence.*;

public class FieldConstants 
{
  /** Length of the field in the X direction, metres */
  public static final double fieldLength = 17.548;
  /** Width of the field in the Y direction, metres */
  public static final double fieldWidth = 8.051;

  /** Distance of the start lines from fieldCentre, metres */
  public static final double startLineOffset = 1.5;

  public static final Translation2d fieldCentre = new Translation2d(fieldLength / 2, fieldWidth / 2);

  public static final ArrayList<Translation2d> blueReefMidpoints = GeoFencing.reefBlue.getMidPoints();
  public static final ArrayList<Translation2d> redReefMidpoints = GeoFencing.reefRed.getMidPoints();
  
  private static final double reefFaceOffset = 0.45;
  private static final Translation2d reefSidewaysOffset = new Translation2d(0, 0.15);

  private static final Translation2d reefCentre = new Translation2d(4.489, fieldCentre.getY());
  private static final Translation2d r1CentreLineup = reefCentre.minus(new Translation2d(reefFaceOffset + (GeoFencing.inscribedReefDiameter / 2), 0));

  public static final Pose2d redStartLine  = new Pose2d(fieldCentre.plus(new Translation2d(startLineOffset, 0)), Rotation2d.kZero);
  public static final Pose2d blueStartLine = new Pose2d(fieldCentre.plus(new Translation2d(-startLineOffset, 0)), Rotation2d.k180deg);

  public static final Pose2d raLineup = new Pose2d(r1CentreLineup.plus(reefSidewaysOffset), Rotation2d.kZero);
  public static final Pose2d rbLineup = new Pose2d(r1CentreLineup.minus(reefSidewaysOffset), Rotation2d.kZero);
  public static final Pose2d rcLineup = raLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(-60)));
  public static final Pose2d rdLineup = rbLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(-60)));
  public static final Pose2d reLineup = raLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(-120)));
  public static final Pose2d rfLineup = rbLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(-120)));
  public static final Pose2d rgLineup = raLineup.rotateAround(reefCentre, Rotation2d.k180deg);
  public static final Pose2d rhLineup = rbLineup.rotateAround(reefCentre, Rotation2d.k180deg);
  public static final Pose2d riLineup = raLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(120)));
  public static final Pose2d rjLineup = rbLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(120)));
  public static final Pose2d rkLineup = raLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(60)));
  public static final Pose2d rlLineup = rbLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(60)));
  public static final Pose2d[] reefLineups = {raLineup, rbLineup, rcLineup, reLineup, rfLineup, rgLineup, rhLineup, riLineup, rjLineup, rkLineup, rlLineup};

  public static Pose2d getLineup(String name)
  {
    return switch (name) 
    {
      case "ra" -> raLineup;
      case "rb" -> rbLineup;
      case "rc" -> rcLineup;
      case "rd" -> rdLineup;
      case "re" -> reLineup;
      case "rf" -> rfLineup;
      case "rg" -> rgLineup;
      case "rh" -> rhLineup;
      case "ri" -> riLineup;
      case "rj" -> rjLineup;
      case "rk" -> rkLineup;
      case "rl" -> rlLineup;
      default -> raLineup;
    };
  }

  public static final double coralStationRange = 0.6;

  public static final class GeoFencing
  {   
    /**
     * Minimum value for object radius, metres </p>
     * The system is not confirmed to handle negative radii
     */
    public static final double minRadius = 0;
    /**
     * Minimum value for object buffer, metres </p>
     * A small buffer is required to ensure safe transitions
     */
    public static final double minBuffer = 0.1;

    // Relative to the centre of the robot, in direction the robot is facing
    // These values are the distance in metres to the virtual wall the robot will stop at
    // 0 means the wall is running through the middle of the robot
    // negative distances will have the robot start outside the area, and can only move into it
    /** Metres the robot can travel left */
    public static final double fieldNorth = fieldWidth;

    /** Metres the robot can travel right */
    public static final double fieldSouth = 0;

    /** Metres the robot can travel forwards */
    public static final double fieldEast = fieldLength;

    /** Metres the robot can travel back */
    public static final double fieldWest = 0;

    /** Buffer zone for the field walls in metres */
    public static final double wallBuffer = 0.5;
    /** Radius for the field walls in metres */
    public static final double wallRadius = 0.05;

    /** Radius from robot centre in metres where geofence is triggered for slow movements */
    public static final double robotRadiusInscribed = 0.35;
    /** Radius from robot centre in metres where geofence is triggered for fast movements */
    public static final double robotRadiusCircumscribed = 0.5;
    /** Radius from robot centre in metres where geofence is triggered for closer approaches */
    public static final double robotRadiusMinimum = 0.25;
    /** Speed threshold at which the robot changes between radii, in meters per second*/
    public static final double robotSpeedThreshold = 1.5;
    
    /** Inscribed diameter of the reef hexagon (i.e. distance between opposite faces) in metres */
    public static final double inscribedReefDiameter = 1.600;
    /** Circumscribed diameter of the reef hexagon (i.e. distance between opposite points) in metres */
    public static final double circumscribedReefDiameter = 1.640;
    /** Circumscribed diameter of the reef zone hexagon (i.e. distance between opposite points) in metres */
    public static final double circumscribedReefZoneDiameter = 3;
    public static final double penaltyReefZoneDiameter = circumscribedReefZoneDiameter + (robotRadiusCircumscribed * 3);
    
    /** Buffer zone for the reef in metres */
    public static final double reefBuffer = 0.5;

    /** Buffer zone for the barge zone in metres */
    public static final double bargeBuffer = 0.5;

    public static final double cornerWidth  = 1.150;
    public static final double cornerLength = 1.620;

    public static final Fence field = new Fence
    (
      fieldWest, 
      fieldSouth, 
      fieldEast, 
      fieldNorth, 
      wallRadius,
      wallBuffer
    );

    public static final Polygon reefBlue      = new Polygon(4.489, 4.026, circumscribedReefDiameter / 2, reefBuffer, 0, 6);
    public static final Polygon reefZoneBlue  = new Polygon(4.489, 4.026, penaltyReefZoneDiameter / 2, reefBuffer, 0, 6);
    public static final Polygon reefRed       = new Polygon(13.059, 4.026, circumscribedReefDiameter / 2, reefBuffer, 180, 6);
    public static final Polygon reefZoneRed   = new Polygon(13.059, 4.026, penaltyReefZoneDiameter / 2, reefBuffer, 180, 6);
    public static final Point   bargeColumn   = new Point(8.774, 4.026, 0.25, 0.15);
    public static final Box     bargeZoneBlue = new Box(8.190, 4.331, 9.358, fieldWidth, 0.1, bargeBuffer);
    public static final Box     bargeZoneRed  = new Box(8.190, 3.721, 9.358, 0, 0.1, bargeBuffer);
    public static final Line    cornerSBlue   = new Line(fieldWest, fieldSouth + cornerWidth, fieldWest + cornerLength, fieldSouth, 0, wallBuffer);
    public static final Line    cornerNBlue   = new Line(fieldWest, fieldNorth - cornerWidth, fieldWest + cornerLength, fieldNorth, 0, wallBuffer);
    public static final Line    cornerSRed    = new Line(fieldEast, fieldSouth + cornerWidth, fieldEast - cornerLength, fieldSouth, 0, wallBuffer);
    public static final Line    cornerNRed    = new Line(fieldEast, fieldNorth - cornerWidth, fieldEast - cornerLength, fieldNorth, 0, wallBuffer);
    public static final Attractor testAttractor = new Attractor(fieldCentre.getX(), fieldCentre.getY(), 0, 5, 1.5);

    // Set up Attractors and Conditions for GeoFence objects
    public static void configureAttractors(BiPredicate<TargetPosition, DriveState> checkTargetAndState)
    {
      reefRed.addRelativeAttractors(0.4, -0.2, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Left, DriveState.Reef)) ;
      reefRed.addRelativeAttractors(0.4, 0, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Centre, DriveState.Reef)) ;
      reefRed.addRelativeAttractors(0.4, 0.2, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Right, DriveState.Reef)) ;
      reefBlue.addRelativeAttractors(0.4, -0.2, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Left, DriveState.Reef)) ;
      reefBlue.addRelativeAttractors(0.4, 0, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Centre, DriveState.Reef)) ;
      reefBlue.addRelativeAttractors(0.4, 0.2, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Right, DriveState.Reef)) ;

      cornerSBlue.addRelativeAttractor(true, 0.4, 0.5, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Left, DriveState.Station)) ;
      cornerSBlue.addRelativeAttractor(true, 0.4, 0, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Centre, DriveState.Station)) ;
      cornerSBlue.addRelativeAttractor(true, 0.4, -0.5, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Right, DriveState.Station)) ;
      
      cornerNBlue.addRelativeAttractor(false, 0.4, 0.5, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Left, DriveState.Station)) ;
      cornerNBlue.addRelativeAttractor(false, 0.4, 0, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Centre, DriveState.Station)) ;
      cornerNBlue.addRelativeAttractor(false, 0.4, -0.5, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Right, DriveState.Station)) ;
      
      cornerSRed.addRelativeAttractor(false, 0.4, 0.5, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Left, DriveState.Station)) ;
      cornerSRed.addRelativeAttractor(false, 0.4, 0, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Centre, DriveState.Station)) ;
      cornerSRed.addRelativeAttractor(false, 0.4, -0.5, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Right, DriveState.Station)) ;
      
      cornerNRed.addRelativeAttractor(true, 0.4, 0.5, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Left, DriveState.Station)) ;
      cornerNRed.addRelativeAttractor(true, 0.4, 0, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Centre, DriveState.Station)) ;
      cornerNRed.addRelativeAttractor(true, 0.4, -0.5, 2.5, 1.2, () -> checkTargetAndState.test(TargetPosition.Right, DriveState.Station)) ;
    }

    public static final ObjectList fieldBlueGeoFence = new ObjectList
    (
      reefBlue, 
      reefZoneRed, 
      bargeColumn, 
      bargeZoneRed,
      cornerSBlue, 
      cornerNBlue
    );

    public static final ObjectList fieldRedGeoFence = new ObjectList
    (
      reefRed, 
      reefZoneBlue, 
      bargeColumn, 
      bargeZoneBlue,
      cornerSRed, 
      cornerNRed
    );

    public static final Fence fieldTest = new Fence
    (
      reefRed.getCentre().getX() - 1.25, 
      fieldCentre.getY() - 3.25,
      reefRed.getCentre().getX() - 9.25, 
      fieldCentre.getY() + 3.25, 
      wallRadius,
      wallBuffer
    );

    public static final ObjectList fieldGeoFence = new ObjectList(fieldTest);//field, fieldBlueGeoFence, fieldRedGeoFence);

    /** Minimum speed limit within a restrictor */
    public static final double minLocalSpeedLimit = 0.05;
  }

  public static final class AutoDrive 
  {
    /** Attractor minimum angle tolerance, degrees */
    public static final double minAngleTolerance = 20;
    /** Attractor maximum angle tolerance, degrees */
    public static final double maxAngleTolerance = 60;
  }
}
