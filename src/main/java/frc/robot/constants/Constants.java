package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

public final class Constants 
{
  public static final class RumblerConstants 
  {
    public static final double driverDefault = 0.1;
    public static final double operatorDefault = 0.1;
  }

  public static final class Control
  {
    public static final double manualDiffectorDeadband = 0.25;
    public static final double stickDeadband = 0.15;
    /** Normal maximum robot speed, relative to maximum uncapped speed */
    public static final double maxThrottle = 0.5;
    /** Minimum robot speed when braking, relative to maximum uncapped speed */
    public static final double minThrottle = 0.3;
    /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
    public static final double maxRotThrottle = 1;
    /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
    public static final double minRotThrottle = 0.5;
    /** Angle tolerance to consider something as "facing" the drivers, degrees */
    public static final double driverVisionTolerance = 5;
    /** Scalar for manual diffector elevation control */
    public static final double manualDiffectorElevationScalar = 2;
    /** Scalar for manual diffector rotation control */
    public static final double manualDiffectorRotationScalar = 2;
    /** Scalar for braking effect of diffector arm being higher than 1m */
    public static final double armBrakeRate = 1.5;
    public static final double manualClimberScale = 1;
    public static final double driveSnappingRange = 1.5;
    public static final double cageFaceDistance = 1.5;
    /** Translation lineup tolerance, in meters */
    public static final double lineupTolerance = 0.05;
    /** Rotation lineup tolerance, in degrees */
    public static final double angleLineupTolerance = 1.5;
  }

  public static final class Swerve
  {
    /** Centre-centre distance (length and width) between wheels, metres */
    public static final double drivebaseWidth = 0.485;
    public static final double initialHeading = 0;

    /* Drive PID Values */
    public static final double driveKP = 2.5;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.12;

    /* Rotation Control PID Values */
    public static final double rotationKP = 6;
    public static final double rotationKI = 0;
    public static final double rotationKD = 0;
    
    /* Rotation Control PID Values when holding Algae */
    public static final double rotationKPAlgae = 5;
    public static final double rotationKIAlgae = 0;
    public static final double rotationKDAlgae = 1;

    /* Swerve Limit Values */
    /** Meters per Second */
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Radians per Second */
    public static final double maxAngularVelocity = 4;
  }

  public static final class Shooter
  {
    public static final double speed = 1.0;

    public static final double slot0S = 0.0;
    public static final double slot0V = 0.0;
    public static final double slot0A = 0.0;
    public static final double slot0P = 0.0;
    public static final double slot0I = 0.0;
    public static final double slot0D = 0.0;

    public static final double velocity = 0.0;
    public static final double acceleration = 0.0;

    public static final double idleSpeed = 0.0;
    public static final double revSpeed = 0.0;
    public static final double leliency = 0.0;

    //simulation
    public static final double kGearRatio = 10.0;
    public static final double kMOI = 0.001; 
  }

  public static final class Vision
  {
    /* public static final int[] validIDs = 
    {
      //1, 2, 3,               // Red Human Player Stations
      //4, 5,                  // Red Barge
      //6, 7, 8, 9, 10, 11,      // Red Reef
      //12, 13, 16,            // Blue Human Player Stations
      //14, 15,                // Blue Barge
      17, 18, 19, 20, 21, 22   // Blue Reef
    };*/

    public static final int[] reefIDs = 
    {
      6, 7, 8, 9, 10, 11,    // Red Reef
      17, 18, 19, 20, 21, 22, // Blue Reef
      1, 2, 3,   // Red Human Player Stations
      12, 13, 16 // Blue Human Player Stations
    };

    public static final int[] bargeIDs = 
    {
      4, 5,  // Red Barge
      14, 15 // Blue Barge
    };

    public static final int[] humanPlayerStationIDs = 
    {
      1, 2, 3,   // Red Human Player Stations
      12, 13, 16 // Blue Human Player Stations
    };

    /** Baseline 1 meter, 1 tag stddev for x and y, in meters */
    public static final double linearStdDevBaseline = 0.08;
    /** Baseline 1 meter, 1 tag stddev rotation, in radians */
    public static final double rotStdDevBaseline = 999;
    /** How many good MT1 readings to get before setting rotation and moving to MT2 */
    public static final int mt1CyclesNeeded = 10;
  }
}
