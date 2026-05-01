package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controlTransmutation.ObjectList;
import frc.robot.controlTransmutation.geoFence.*;
import frc.robot.controlTransmutation.triggerObject.*;
import frc.robot.util.AlliancePose2d;

import static frc.robot.constants.Constants.SwerveConstants.robotRadiusExpanded;
import static frc.robot.constants.Constants.SwerveConstants.robotRadiusInscribed;

/**
 * Geometry data for "Rebuilt" field
 * @author 5985
 */
public class FieldConstants 
{
  /** Adjustment values to account for the physical field */
  public static final class FieldTuning
  {
    /** Offset for climb lineup, metres away from driverstation wall */
    public static final double climbOffsetBlueRight = -0.034;
    /** Offset for climb lineup, metres away from driverstation wall */
    public static final double climbOffsetBlueLeft  = -0.023;
    /** Offset for climb lineup, metres away from driverstation wall */
    public static final double climbOffsetRedRight  = -0.033;
    /** Offset for climb lineup, metres away from driverstation wall */
    public static final double climbOffsetRedLeft   = -0.03;
    
    /** Offset for climb lineup, degrees counterclockwise from nominal angle */
    public static final double climbAngleBlueRight  = -0.1;
    /** Offset for climb lineup, degrees counterclockwise from nominal angle */
    public static final double climbAngleBlueLeft   = 0.0;
    /** Offset for climb lineup, degrees counterclockwise from nominal angle */
    public static final double climbAngleRedRight   = 0.0;
    /** Offset for climb lineup, degrees counterclockwise from nominal angle */
    public static final double climbAngleRedLeft    = 0.7;

    /** Climber max position, metres above climber 0-position */
    public static final double postHeightOffsetBlueLeft  = 0.230;
    /** Climber max position, metres above climber 0-position */
    public static final double postHeightOffsetRedLeft   = 0.230;
    /** Climber max position, metres above climber 0-position */
    public static final double postHeightOffsetBlueRight = 0.227;
    /** Climber max position, metres above climber 0-position */
    public static final double postHeightOffsetRedRight  = 0.230;
  }

  public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 

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
    public static final double wallBuffer = 1;
    /** Radius around field walls, metres */
    public static final double wallRadius = 0.0;
    
    /** Radius around hubs, metres */
    public static final double hubRadius = 0.15;
    /** Buffer zone around hubs, metres */
    public static final double hubBuffer = 0.5;

    public static final double hubSideLength = 1.19;

    public static final double hubYa = fieldCentre.getY() - hubSideLength / 2;
    public static final double hubYb = fieldCentre.getY() + hubSideLength / 2;

    /* How far from the field center line the hub front/back is offset */
    public static final double hubFrontOffset = hubCentreOffset + hubSideLength / 2;
    public static final double hubBackOffset  = hubCentreOffset - hubSideLength / 2;

    public static final double hubOutputDepth = 2;

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

    //public static final Point hubBlueOutput = new Point(fieldCentre.getX() - hubBackOffset, fieldCentre.getY(), hubSideLength / 2, hubBuffer);
    //public static final Point hubRedOutput  = new Point(fieldCentre.getX() - hubBackOffset, fieldCentre.getY(), hubSideLength / 2, hubBuffer);

    private static final double obstacleRadius = 0.0;

    public static final BoxRegion obstacleBlue = new BoxRegion(fieldCentre.getX() - hubFrontOffset, 0, fieldCentre.getX() - hubBackOffset, fieldWidth, obstacleRadius, 0);
    public static final BoxRegion obstacleRed  = new BoxRegion(fieldCentre.getX() + hubFrontOffset, 0, fieldCentre.getX() + hubBackOffset, fieldWidth, obstacleRadius, 0);

    /* Bump Zone */
    // Speed should be limited when traversing
    // Rotation must NOT be square when traversing
    /** Throttle limit when within bump zone */
    //public static final double bumpSpeedLimit = 0;
    public static final double bumpRotationTolerance = 40;
    public static final double bumpWidth = 1.85;
    public static final double columnBumpBuffer = 0.65;
    public static final double bumpYa = hubYa - bumpWidth;
    public static final double bumpYb = hubYb + bumpWidth;

    public static final double bumpDepth = 0.75;
    public static final double bumpXa = hubCentreOffset + bumpDepth/2;
    public static final double bumpXb = hubCentreOffset - bumpDepth/2;

    public static final BoxRegion bumpSB = new BoxRegion(fieldCentre.getX() - bumpXa, bumpYa + columnBumpBuffer, fieldCentre.getX() - bumpXb, hubYa);
    public static final BoxRegion bumpNB = new BoxRegion(fieldCentre.getX() - bumpXa, hubYb,  fieldCentre.getX() - bumpXb, bumpYb - columnBumpBuffer);
    public static final BoxRegion bumpSR = new BoxRegion(fieldCentre.getX() + bumpXa, bumpYa + columnBumpBuffer, fieldCentre.getX() + bumpXb, hubYa);
    public static final BoxRegion bumpNR = new BoxRegion(fieldCentre.getX() + bumpXa, hubYb,  fieldCentre.getX() + bumpXb, bumpYb - columnBumpBuffer);

    public static final Trigger bumpTrigger = 
          bumpSB.asTrigger()
      .or(bumpNB.asTrigger())
      .or(bumpSR.asTrigger())
      .or(bumpNR.asTrigger());
    
    /* Trench Zone */
    public static final double trenchWidth = 1.28;
    public static final double trenchEffectWidth = trenchWidth / 2;
    /** Depth of region either side of Trench bar to trigger nudging */
    public static final double trenchZoneDepth = 1.4;
    public static final double trenchXa = hubCentreOffset + trenchZoneDepth;
    public static final double trenchXb = hubCentreOffset - trenchZoneDepth;

    public static final BoxRegion trenchSB = new BoxRegion(fieldCentre.getX() - trenchXa, 0, fieldCentre.getX() - trenchXb, trenchEffectWidth);
    public static final BoxRegion trenchNB = new BoxRegion(fieldCentre.getX() - trenchXa, fieldWidth - trenchEffectWidth, fieldCentre.getX() - trenchXb, fieldWidth);
    public static final BoxRegion trenchSR = new BoxRegion(fieldCentre.getX() + trenchXa, 0, fieldCentre.getX() + trenchXb, trenchEffectWidth);
    public static final BoxRegion trenchNR = new BoxRegion(fieldCentre.getX() + trenchXa, fieldWidth - trenchEffectWidth, fieldCentre.getX() + trenchXb, fieldWidth);

    public static final Trigger trenchTrigger = 
          trenchSB.asTrigger()
      .or(trenchNB.asTrigger())
      .or(trenchSR.asTrigger())
      .or(trenchNR.asTrigger());

    /* Trench Column */
    public static final double trenchColumnDepth = 1.3;
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
    /** Radius around the Tower base to avoid, m */
    public static final double towerBaseRadius = 0.3;
    /** Radius to treat Tower uprights as circles, m */
    public static final double towerPostRadius = 0.2;
    /** Distance from edge of Tower base to centre of upright, m */
    public static final double towerPostEdge   = 0.06;
    /** Distance from front of Tower base to centre of upright, m */
    public static final double towerPostFront  = 0.07;

    public static final double towerPostBlueX = towerDepth - towerPostFront;
    public static final double towerPostBlueRightY = towerSpacing + towerPostEdge;
    public static final double towerPostBlueLeftY = towerSpacing + towerWidth - towerPostEdge;
    public static final double towerPostRedX = fieldLength - (towerDepth - towerPostFront);
    public static final double towerPostRedRightY = fieldWidth - (towerSpacing + towerPostEdge);
    public static final double towerPostRedLeftY = fieldWidth - (towerSpacing + towerWidth - towerPostEdge);

    public static final Box towerBlue = new Box(0, towerPostBlueRightY, towerPostBlueX, towerPostBlueLeftY, towerBaseRadius, 0.25);
    public static final Point towerPostBlueN = new Point(towerPostBlueX, towerPostBlueLeftY, towerPostRadius, 0.25);
    public static final Point towerPostBlueS = new Point(towerPostBlueX, towerPostBlueRightY, towerPostRadius, 0.25);
    
    public static final Box towerRed = new Box(fieldLength, towerPostRedLeftY, towerPostRedX, towerPostRedRightY, towerBaseRadius, 0.25);
    public static final Point towerPostRedN = new Point(towerPostRedX, towerPostRedRightY, towerPostRadius, 0.25);
    public static final Point towerPostRedS = new Point(towerPostRedX, towerPostRedLeftY, towerPostRadius, 0.25);

    /* Tower Exclusion Zones */
    // Region in which turrets cannot safely shoot
    // Region should technically be trapezoidal, but this is using a square region to simplify computation
    /** Distance from right edge of Tower base to prevent shooting, m */
    public static final double towerShadowRight = 0.3;
    /** Distance from left edge of Tower base to prevent shooting, m */
    public static final double towerShadowLeft = 0.2;

    public static final BoxRegion towerShadowBlue = new BoxRegion(0, towerPostBlueRightY - towerShadowRight, towerPostBlueX, towerPostBlueLeftY + towerShadowLeft);
    public static final BoxRegion towerShadowRed  = new BoxRegion(fieldLength, towerPostRedRightY + towerShadowRight, towerPostRedX, towerPostRedLeftY - towerShadowLeft);
    
    // Regions to avoid other climbing robots
    /** Assumed radius for other robots, m */
    private static final double clearRadius = 0.45;
    private static final double climbAllowance = clearRadius + 0.3;
    public static final Box towerClearBlueLeft  = new Box(0, towerPostBlueRightY + climbAllowance, towerPostBlueX + clearRadius, towerPostBlueLeftY + clearRadius, clearRadius, climbAllowance);
    public static final Box towerClearBlueRight = new Box(0, towerPostBlueRightY - clearRadius, towerPostBlueX + clearRadius, towerPostBlueLeftY - climbAllowance, clearRadius, climbAllowance);
    public static final Box towerClearRedLeft   = new Box(fieldLength, towerPostRedLeftY - clearRadius, towerPostRedX - clearRadius, towerPostRedRightY - climbAllowance, clearRadius, climbAllowance);
    public static final Box towerClearRedRight  = new Box(fieldLength, towerPostRedLeftY + climbAllowance, towerPostRedX - clearRadius, towerPostRedRightY + clearRadius, clearRadius, climbAllowance);

    /** Speed limit when lining up to climb, [0..1] */
    private static final double climbSlowLimit  = 0.4;
    public static final BoxRegion climbSlowBlue = new BoxRegion(0, 0, towerDepth, fieldWidth, 0, 1);
    public static final BoxRegion climbSlowRed  = new BoxRegion(fieldLength, 0, fieldLength - towerDepth, fieldWidth, 0, 1);
    
    // Prevent the robot from coming to close to driver wall when lining up to climb
    /** Depth of barrier from driver wall when climbing, metres */
    private static final double climbWallDepth  = towerPostBlueX - robotRadiusExpanded - 0.1;
    public static final Box climbWallBlue       = new Box(0, 0, climbWallDepth, fieldWidth);
    public static final Box climbWallRed        = new Box(fieldLength, 0, fieldLength - climbWallDepth, fieldWidth);

    // Prevent the robot from coming sideways into the post
    public static final Box towerClearBlue      = new Box(towerPostBlueX + climbAllowance, towerPostBlueRightY, towerPostBlueX + climbAllowance, towerPostBlueLeftY, 0, 0.25);
    public static final Box towerClearRed       = new Box(towerPostRedX - climbAllowance, towerPostRedRightY, towerPostRedX - climbAllowance, towerPostRedLeftY, 0, 0.25);
    
    public static final ObjectList climbBarrier = new ObjectList
    (
      towerClearBlue,
      towerClearRed,
      climbWallBlue,
      climbWallRed,
      climbSlowBlue.withSpeedLimit(climbSlowLimit),
      climbSlowRed.withSpeedLimit(climbSlowLimit)
    );

    // TriggerVectors for climbing
    /** Distance from trigger to activate lineup sequence, metres */
    private static final double climbTriggerRadius = 2.5;
    /** Disatance from robot radius to post to allow manual adjustment, metres */
    private static final double climbTriggerBuffer = 0.1;
    /** Distance past post to centre the trigger to account for difference between robot radius and climber contact point, metres */
    private static final double climbTriggerOffset = 0.2;
    public static final Translation2d climbApproachOffset = new Translation2d(0, 1.2);
    public static final Translation2d climbStartOffset = new Translation2d(0, 0.7);
    public static final Translation2d climbEndOffset = new Translation2d(0, 0.35);

    public static final TriggerVector climbBlueRight = new 
      TriggerVector(towerPostBlueX + FieldTuning.climbOffsetBlueRight, towerPostBlueRightY + climbTriggerOffset, 90, climbTriggerRadius, climbTriggerBuffer)
      .withBufferActivation(true);
    public static final TriggerVector climbBlueLeft  = new 
      TriggerVector(towerPostBlueX + FieldTuning.climbOffsetBlueLeft, towerPostBlueLeftY - climbTriggerOffset, -90, climbTriggerRadius, climbTriggerBuffer)
      .withBufferActivation(true);
    public static final TriggerVector climbRedRight  = new 
      TriggerVector(towerPostRedX - FieldTuning.climbOffsetRedRight, towerPostRedRightY - climbTriggerOffset, -90, climbTriggerRadius, climbTriggerBuffer)
      .withBufferActivation(true);
    public static final TriggerVector climbRedLeft   = new 
      TriggerVector(towerPostRedX - FieldTuning.climbOffsetRedLeft, towerPostRedLeftY + climbTriggerOffset, 90, climbTriggerRadius, climbTriggerBuffer)
      .withBufferActivation(true);

    public static final ObjectList climbTriggerVectors = new ObjectList
    (
      climbBlueRight,
      climbBlueLeft,
      climbRedRight,
      climbRedLeft
    );

    public static final AlliancePose2d climbStartPoseRight = new AlliancePose2d(towerPostBlueS.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCW_90deg);
    public static final AlliancePose2d climbEndPoseRight = new AlliancePose2d(towerPostBlueS.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCW_90deg);
    public static final AlliancePose2d climbStartPoseLeft = new AlliancePose2d(towerPostBlueN.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCCW_90deg);
    public static final AlliancePose2d climbEndPoseLeft = new AlliancePose2d(towerPostBlueN.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCCW_90deg);

    /* Depot */
    // Speed should be limited in own Depot, must NOT enter opposing
    public static final double depotWidth = 1.07;
    public static final double depotDepth = 0.69;
    public static final double depotSpacing = 1.58;
    
    public static final Box depotBlueFence = new Box(0, fieldWidth - depotSpacing, depotDepth, fieldWidth - (depotSpacing + depotWidth), 0.1, 0.25);
    public static final Box depotRedFence  = new Box(fieldLength, depotSpacing, fieldLength - depotDepth, depotSpacing + depotWidth, 0.1, 0.25);
    
    public static final BoxRegion depotBlueZone = new BoxRegion(0, fieldWidth - depotSpacing, depotDepth, fieldWidth - (depotSpacing + depotWidth));
    public static final BoxRegion depotRedZone  = new BoxRegion(fieldLength, depotSpacing, fieldLength - depotDepth, depotSpacing + depotWidth);

    public static final ObjectList fieldStaticGeoFence = new ObjectList
    (
      bumpSB,
      bumpNB,
      bumpSR,
      bumpNR,
      trenchColSB,
      trenchColNB,
      trenchColSR,
      trenchColNR,
      trenchSB,
      trenchNB,
      trenchSR,
      trenchNR,
      hubBlue, 
      hubRed
    );

    public static final ObjectList fieldBlueGeoFence = new ObjectList
    (
      towerPostBlueN,
      towerPostBlueS,
      towerClearBlueLeft,
      towerClearBlueRight,
      towerRed
    );

    public static final ObjectList fieldRedGeoFence = new ObjectList
    (
      towerPostRedN,
      towerPostRedS,
      towerClearRedLeft,
      towerClearRedRight,
      towerBlue
    );

    public static final ObjectList fieldGeoFence = new ObjectList
    (
      fieldStaticGeoFence, 
      fieldBlueGeoFence, 
      fieldRedGeoFence,
      climbBarrier
    ).addPriority(field);

    /** Minimum speed limit within a restrictor */
    public static final double minLocalSpeedLimit = 0.05;
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
