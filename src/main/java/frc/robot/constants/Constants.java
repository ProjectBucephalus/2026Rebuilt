package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;

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
    /** Translation lineup tolerance, in meters */
    public static final double lineupTolerance = 0.05;
    /** Rotation lineup tolerance, in degrees */
    public static final double angleLineupTolerance = 3;
  }

  public static final class Swerve
  {
    /** Forward offset between the centre of the drivebase and Robot coordinate origin, metres */
    public static final double drivebaseOffset = 0.095;
    /** Centre-centre distance between wheels, metres */
    public static final double drivebaseWidth = 0.485;
    /** Centre-centre distance between wheels, metres */
    public static final double drivebaseLength = drivebaseWidth;
    /** Wheel-centre to Robot-centre distance to Port wheels, metres */
    public static final double wheelPortY = drivebaseWidth/2;
    /** Wheel-centre to Robot-centre distance to Stbd wheels, metres */
    public static final double wheelStbdY = -drivebaseWidth/2;
    /** Wheel-centre to Robot-centre distance to Fore wheels, metres */
    public static final double wheelForeX = (drivebaseLength/2) - drivebaseOffset;
    /** Wheel-centre to Robot-centre distance to Aft wheels, metres */
    public static final double wheelAftX = (-drivebaseLength/2) - drivebaseOffset;


    public static final double initialHeading = 0;
    /** Offset from centre of drivebase to use as centre of rotation when extended, metres Fore/Port */
    public static final Translation2d extendedCentreOffset = new Translation2d(drivebaseOffset,0);

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
    public static final double slot0P = 1.0;
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

    public static final int[] hubIDs = 
    {
      /* RED */ 
      3, 4, // Inner
      9, 10, // Outer
      5, 8, // Scoring Side
      11, 2, // Non-Scoring Side

      /* BLUE */ 
      19, 20, // Inner
      25, 26, // Outer
      18, 27, // Scoring Side
      21, 24 // Non-Scoring Side
    };

    public static final int[] towerIDs = 
    {
      /* RED */
      15, 16,

      /* BLUE */
      31, 32
    };

    public static final int[] outpostIDs = 
    {
      /* RED */
      13, 14,

      /* BLUE */
      29, 30
    };

    public static final int[] trenchIDs = 
    {
      /* RED */
      6, 7, // Scoring Side
      1, 12, // Non-Scoring Side

      /* BLUE */
      17, 28, // Scoring Side
      22, 23 // Non-Scoring Side
    };

    public static final int[] allIDs;
    static 
    {
      allIDs = new int[hubIDs.length + towerIDs.length + outpostIDs.length + trenchIDs.length];

      // Combine all of the ID arrays efficiently with checked memcpys
      System.arraycopy(hubIDs, 0, allIDs, 0, hubIDs.length);
      System.arraycopy(towerIDs, 0, allIDs, hubIDs.length, towerIDs.length);
      System.arraycopy(outpostIDs, 0, allIDs, hubIDs.length + towerIDs.length, outpostIDs.length);
      System.arraycopy(trenchIDs, 0, allIDs, hubIDs.length + towerIDs.length + outpostIDs.length, trenchIDs.length);
    }

    /** Baseline 1 meter, 1 tag stddev for x and y, in meters */
    public static final double linearStdDevBaseline = 0.08;
    /** Baseline 1 meter, 1 tag stddev rotation, in radians */
    public static final double rotStdDevBaseline = 0.5;
    /** How many good MT1 readings to get before setting rotation and moving to MT2 */
    public static final int mt1CyclesNeeded = 10;
  }
public static final class Turret
  {
    public static final double maxTurretAzimuth = 270;
    public static final double gearRatio = 7;
    public static final double turretAcceleration = 1;
    public static final double turretVelocity = 1;  
    public static final double turretIdlePosition = 0;
    public static final double turretTurnSpeed = 0.25;
    public static final double turnBackThreshold = 135;
  }


}
