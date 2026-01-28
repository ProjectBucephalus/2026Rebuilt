package frc.robot.constants;

import static frc.robot.constants.FieldConstants.GeoFencing.robotRadiusExpanded;
import static frc.robot.constants.FieldConstants.GeoFencing.robotRadiusInscribed;

import java.util.function.BiPredicate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot.DriveState;
import frc.robot.Robot.TargetPosition;
import frc.robot.util.controlTransmutation.Attractor;
import frc.robot.util.controlTransmutation.ObjectList;
import frc.robot.util.controlTransmutation.geoFence.*;
import frc.robot.util.controlTransmutation.restrictor.*;

public class FieldConstants 
{
  /** Length of the field in the X direction, metres */
  public static final double fieldLength = 16.54;
  /** Width of the field in the Y direction, metres */
  public static final double fieldWidth = 8.08;

  /** Distance of the start lines from fieldCentre, metres */
  public static final double startLineOffset = 4.2418;

  public static final Translation2d fieldCentre = new Translation2d(fieldLength / 2, fieldWidth / 2);

  public static final Pose2d redStartLine  = new Pose2d(fieldCentre.plus(new Translation2d((startLineOffset + robotRadiusInscribed), 0)), Rotation2d.kZero);
  public static final Pose2d blueStartLine = new Pose2d(fieldCentre.plus(new Translation2d(-(startLineOffset + robotRadiusInscribed), 0)), Rotation2d.k180deg);

  public static final double hubCentreOffset = 3.645;

  public static final Translation2d redHubCentre = new Translation2d(fieldCentre.getX() + hubCentreOffset, fieldCentre.getY());
  public static final Translation2d blueHubCentre = new Translation2d(fieldCentre.getX() - hubCentreOffset, fieldCentre.getY());

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
    
    /** Metres the robot can travel from Scoring Table */
    public static final double fieldNorth = fieldWidth;

    /** Metres the robot can travel towards Scoring Table */
    public static final double fieldSouth = 0;

    /** Metres the robot can travel towards Red */
    public static final double fieldEast = fieldLength;

    /** Metres the robot can travel towards Blue */
    public static final double fieldWest = 0;

    /** Buffer zone around field walls, metres */
    public static final double wallBuffer = 0.5;
    /** Radius around field walls, metres */
    public static final double wallRadius = 0.05;

    /** Radius from robot centre in metres where geofence is triggered for slow movements */
    public static final double robotRadiusInscribed = 0.42;
    /** Radius from robot centre in metres where geofence is triggered for fast movements */
    public static final double robotRadiusCircumscribed = 0.6;
    /** Radius enclosing robot when extended in metres where geofence is triggered for most movement */
    public static final double robotRadiusExpanded = 0.66;
    /** Speed threshold at which the robot changes between radii, m/s */
    public static final double robotSpeedThreshold = 1.5;
    
    /** Radius around hubs, metres */
    public static final double hubRadius = 0.05;
    /** Buffer zone around hubs, metres */
    public static final double hubBuffer = 0.5;

    public static final double hubSideLength = 1.19;

    public static final double hubYa = fieldCentre.getY() - hubSideLength / 2;
    public static final double hubYb = fieldCentre.getY() + hubSideLength / 2;

    /* How far from the field center line the hub front/back is offset */
    public static final double hubFrontOffset = hubCentreOffset + hubSideLength / 2;
    public static final double hubBackOffset  = hubCentreOffset - hubSideLength / 2;

    public static final Fence field = new Fence
    (
      fieldWest, 
      fieldSouth, 
      fieldEast, 
      fieldNorth, 
      wallRadius,
      wallBuffer
    );

    /*
      |                               |
      |                               |
      B      |====B       B====|      R
      L------|    |---+---|    |------E
      U      A====|       |====A      D
      | ^                             |
      | 0 >                           |
     */
    public static final Box hubBlue = new Box(fieldCentre.getX() - hubFrontOffset, hubYa, fieldCentre.getX() - hubBackOffset, hubYb, hubRadius, hubBuffer);
    public static final Box hubRed  = new Box(fieldCentre.getX() + hubFrontOffset, hubYa, fieldCentre.getX() + hubBackOffset, hubYb, hubRadius, hubBuffer);

    /* Bump Zone */
    // Speed should be limited when traversing
    // Rotation must NOT be square when traversing
    /** Throttle limit when within bump zone */
    public static final double bumpSpeedLimit = 0.6;
    public static final double bumpWidth = 1.85;
    public static final double bumpYa = hubYa - bumpWidth;
    public static final double bumpYb = hubYb + bumpWidth;

    public static final double bumpDepth = 1.13;
    public static final double bumpXa = hubCentreOffset + bumpDepth/2;
    public static final double bumpXb = hubCentreOffset - bumpDepth/2;

    public static final BoxRegion bumpSB = new BoxRegion(fieldCentre.getX() - bumpXa, bumpYa, fieldCentre.getX() - bumpXb, hubYa);
    public static final BoxRegion bumpNB = new BoxRegion(fieldCentre.getX() - bumpXa, hubYb,  fieldCentre.getX() - bumpXb, bumpYb);
    public static final BoxRegion bumpSR = new BoxRegion(fieldCentre.getX() + bumpXa, bumpYa, fieldCentre.getX() + bumpXb, hubYa);
    public static final BoxRegion bumpNR = new BoxRegion(fieldCentre.getX() + bumpXa, hubYb,  fieldCentre.getX() + bumpXb, bumpYb);
    
    /* Trench Zone */
    // Rotation MUST be within tollerance when traversing
    // Climber and turret hoods MUST be stowed when traversing
    public static final double trenchWidth = 1.28;
    /** Depth of region around Trench bar to keep out of */
    public static final double trenchBarrierDepth = 0.8;
    public static final double trenchXa = hubCentreOffset + trenchBarrierDepth/2;
    public static final double trenchXb = hubCentreOffset - trenchBarrierDepth/2;

    public static final Box trenchSB = new Box(fieldCentre.getX() - trenchXa, 0, fieldCentre.getX() - trenchXb, trenchWidth);
    public static final Box trenchNB = new Box(fieldCentre.getX() - trenchXa, fieldWidth - trenchWidth, fieldCentre.getX() - trenchXb, fieldWidth);
    public static final Box trenchSR = new Box(fieldCentre.getX() + trenchXa, 0, fieldCentre.getX() + trenchXb, trenchWidth);
    public static final Box trenchNR = new Box(fieldCentre.getX() + trenchXa, fieldWidth - trenchWidth, fieldCentre.getX() + trenchXb, fieldWidth);

    
    /* Trench Column */
    //public static final double trenchColumnWidth = 1.67 - trenchWidth;
    public static final double trenchColumnDepth = 1.2;
    public static final double trenchColXa = hubCentreOffset + trenchColumnDepth/2;
    public static final double trenchColXb = hubCentreOffset - trenchColumnDepth/2;

    public static final Box trenchColSB = new Box(fieldCentre.getX() - trenchColXa, bumpYa, fieldCentre.getX() - trenchColXb, trenchWidth);
    public static final Box trenchColNB = new Box(fieldCentre.getX() - trenchColXa, fieldWidth - trenchWidth, fieldCentre.getX() - trenchColXb, bumpYb);
    public static final Box trenchColSR = new Box(fieldCentre.getX() + trenchColXa, bumpYa, fieldCentre.getX() + trenchColXb, trenchWidth);
    public static final Box trenchColNR = new Box(fieldCentre.getX() + trenchColXa, fieldWidth - trenchWidth, fieldCentre.getX() + trenchColXb, bumpYb);


    /* Tower */
    // Keep clear of opposing tower, keep safe from own posts
    /** Width of Tower base, m */
    public static final double towerWidth = 0.99;
    /** Depth of Tower base, m */
    public static final double towerDepth = 1.15;
    /** Distance from Outpost wall to Tower base, m */
    public static final double towerSpacing = 3.26;
    /** Radius to treat Tower uprights as circles, m */
    public static final double towerPostRadius = 0.1;
    /** Distance from edge of Tower base to centre of upright, m */
    public static final double towerPostEdge   = 0.06;
    /** Distance from front of Tower base to centre of upright, m */
    public static final double towerPostFront  = 0.08;

    public static final Box towerBlue = new Box(0, towerSpacing, towerDepth, towerSpacing + towerWidth);
    public static final Point towerPostBlueN = new Point(towerDepth - towerPostFront, towerSpacing + towerWidth - towerPostEdge, towerPostRadius, 0.25);
    public static final Point towerPostBlueS = new Point(towerDepth - towerPostFront, towerSpacing + towerPostEdge, towerPostRadius, 0.25);
    
    public static final Box towerRed = new Box(fieldLength, fieldWidth - towerSpacing, fieldLength - towerDepth, fieldWidth - (towerSpacing + towerWidth));
    public static final Point towerPostRedN = new Point(fieldLength - (towerDepth - towerPostFront), fieldWidth - (towerSpacing + towerWidth - towerPostEdge), towerPostRadius, 0.25);
    public static final Point towerPostRedS = new Point(fieldLength - (towerDepth - towerPostFront), fieldWidth - (towerSpacing + towerPostEdge), towerPostRadius, 0.25);


    /* Depot */
    // Speed should be limited in own Depot, must NOT enter opposing
    public static final double depotWidth = 1.07;
    public static final double depotDepth = 0.69;
    public static final double depotSpacing = 1.58;
    
    public static final Box depotBlueFence = new Box(0, fieldWidth - depotSpacing, depotDepth, fieldWidth - (depotSpacing + depotWidth), 0.1, 0.25);
    public static final Box depotRedFence  = new Box(fieldLength, depotSpacing, fieldLength - depotDepth, depotSpacing + depotWidth, 0.1, 0.25);
    
    public static final BoxRegion depotBlueZone = new BoxRegion(0, fieldWidth - depotSpacing, depotDepth, fieldWidth - (depotSpacing + depotWidth));
    public static final BoxRegion depotRedZone  = new BoxRegion(fieldLength, depotSpacing, fieldLength - depotDepth, depotSpacing + depotWidth);

    // Set up Attractors and Conditions for GeoFence objects
    public static void configureAttractors(BiPredicate<TargetPosition, DriveState> checkTargetAndState)
    {

    }

    public static final ObjectList fieldStaticGeoFence = new ObjectList
    (
      bumpSB.withSpeedLimit(bumpSpeedLimit),
      bumpNB.withSpeedLimit(bumpSpeedLimit),
      bumpSR.withSpeedLimit(bumpSpeedLimit),
      bumpNR.withSpeedLimit(bumpSpeedLimit),
      trenchColSB,
      trenchColNB,
      trenchColSR,
      trenchColNR,
      hubBlue, 
      hubRed
    );

    public static final ObjectList fieldBlueGeoFence = new ObjectList
    (
      towerPostBlueN,
      towerPostBlueS,
      towerRed,
      depotBlueZone,
      depotRedFence
    );

    public static final ObjectList fieldRedGeoFence = new ObjectList
    (
      towerPostRedN,
      towerPostRedS,
      towerBlue,
      depotRedZone,
      depotBlueFence
    );

    public static final ObjectList fieldGeoFence = new ObjectList(fieldStaticGeoFence, fieldBlueGeoFence, fieldRedGeoFence)
                                                        .addPriority(field);

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

/*=========BLUE========+====================+====================+==========RED=========\
|                      T                    |                    T                      O
|                      r                                         r                      u
|==+                   e                    |                    e                      t
Dep|                 +=+=+                                     +=+=+                    |
|==+                 | B |                  |                  | B |                    |
|                    | m |                                     | m |               +====|
|                    | p |                  |                  | p |               |    |
|====+               +===+                                     +===+               |Tower
|    |               |Hub|                  +                  |Hub|               |    |
Tower|               +===+                                     +===+               +====|
|    |               | B |                  |                  | B |                    |
|====+               | m |                                     | m |                    |
|                    | p |                  |                  | p |                 +==|
|    ^               +=+=+                                     +=+=+                 |Dep
O    Y                 T                    |                    T                   +==|
u    0 X >             r                                         r                      |
t                      e                    |                    e                      |
\======================+====================+====================+=====================*/
