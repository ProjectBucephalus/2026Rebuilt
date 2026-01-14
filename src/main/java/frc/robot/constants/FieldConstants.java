package frc.robot.constants;

import java.util.function.BiPredicate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot.DriveState;
import frc.robot.Robot.TargetPosition;
import frc.robot.util.controlTransmutation.Attractor;
import frc.robot.util.controlTransmutation.ObjectList;
import frc.robot.util.controlTransmutation.geoFence.*;

public class FieldConstants 
{
  /** Length of the field in the X direction, metres */
  public static final double fieldLength = 16.54;
  /** Width of the field in the Y direction, metres */
  public static final double fieldWidth = 8.07;

  /** Distance of the start lines from fieldCentre, metres */
  public static final double startLineOffset = 4.2418;

  public static final Translation2d fieldCentre = new Translation2d(fieldLength / 2, fieldWidth / 2);

  public static final Pose2d redStartLine  = new Pose2d(fieldCentre.plus(new Translation2d(startLineOffset, 0)), Rotation2d.kZero);
  public static final Pose2d blueStartLine = new Pose2d(fieldCentre.plus(new Translation2d(-startLineOffset, 0)), Rotation2d.k180deg);

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
    
    /** Buffer zone for the hubs in metres */
    public static final double hubBuffer = 0.5;

    public static final double hubSideLength = 1.19;

    public static final double hubYa = fieldCentre.getY() - hubSideLength / 2;
    public static final double hubYb = fieldCentre.getY() + hubSideLength / 2;

    /* How far from the field center line the hub centre/front/back is offset */
    public static final double hubCentreOffset = 3.645;
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
      |                              |
      |                              |
      B      |====B      B====|      R
      L------|    |------|    |------E
      U      A====|      |====A      D
      |                              |
      |                              |
     */
    public static final Box hubBlue = new Box(fieldCentre.getX() - hubFrontOffset, hubYa, fieldCentre.getX() - hubBackOffset, hubYb, 0.1, hubBuffer);
    public static final Box hubRed  = new Box(fieldCentre.getX() + hubFrontOffset, hubYa, fieldCentre.getX() + hubBackOffset, hubYb, 0.1, hubBuffer);

    // Set up Attractors and Conditions for GeoFence objects
    public static void configureAttractors(BiPredicate<TargetPosition, DriveState> checkTargetAndState)
    {

    }

    public static final ObjectList fieldBlueGeoFence = new ObjectList
    (
      hubBlue, 
      hubRed
    );

    public static final ObjectList fieldRedGeoFence = new ObjectList
    (
      hubBlue, 
      hubRed
    );

    public static final ObjectList fieldGeoFence = new ObjectList(field);//, fieldBlueGeoFence, fieldRedGeoFence);

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
