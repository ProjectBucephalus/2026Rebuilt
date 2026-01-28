package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Transform3d;
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
    /*
     * To tune shooter:
     *    Find voltage KS required to overcome static friction
     *    Run with voltage at maximum safe limit, record voltage and RPS
     *    Set voltage KV as voltage/RPS
     *    Once KV is tuned, use KP for additional gain as needed
     */
    public static final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration(); 
    {
      flywheelConfig.Slot0.kS = 0.2;
      flywheelConfig.Slot0.kV = 0.08;
      flywheelConfig.Slot0.kA = 0.0;
      flywheelConfig.Slot0.kP = 0.0;
      flywheelConfig.Slot0.kI = 0.0;
      flywheelConfig.Slot0.kD = 0.0;

      flywheelConfig.MotionMagic.MotionMagicAcceleration = 50.0;
      flywheelConfig.MotionMagic.MotionMagicJerk = 50.0;
    }

    public static final double idleSpeed = 10.0;
    public static final double revSpeed = 50.0;
    public static final double leliency = 5.0;

    public static final double flySpeedTolerance = 100;

    //simulation
    public static final double kGearRatio = 10.0;
    public static final double kMOI = 0.001; 

    public static final class HoodConstants 
    {
      public static final double altTolerance = 3;

      public static final double servoRange = 270;
      public static final double servoGear = 20;
      public static final double hoodGear = 193;
      public static final double hoodRange = 25;
      public static final double hoodRatio = servoGear / hoodGear;
    }

    public static final class TurretConstants
    {
      public static final double maxTurretAzimuth = 270;
      
      public static final double turretIdlePosition = 0;

      public static final double turretTurnSpeed = 0.25;

      public static final double potRange = 3600;
      public static final double potPortOffset = -1800;
      public static final double potStbdOffset = -1800;
      public static final double potGear = 20;
      public static final double turretGear = 90;
      public static final double azimuthGearRatio = potGear / turretGear;

      public static final double azimuthTolerance = 3;
      public static final double maxRPM = 2000;
      public static final double limitBufferZone = 10;
      

      public static final TalonFXConfiguration turretConfigs = new TalonFXConfiguration();
      
      static 
      {
        turretConfigs.Slot0.kS = 0.0;
        turretConfigs.Slot0.kV = 0.0;
        turretConfigs.Slot0.kA = 0.0;
        turretConfigs.Slot0.kP = 10.0;
        turretConfigs.Slot0.kI = 0.0;
        turretConfigs.Slot0.kD = 0.0;

        turretConfigs.MotionMagic.MotionMagicAcceleration = 1;
        turretConfigs.MotionMagic.MotionMagicCruiseVelocity = turretTurnSpeed;
      }
    }
  }

  public static final class Vision
  {
    public static final Transform3d portLimelightOffset = new Transform3d();
    public static final Transform3d stbdLimelightOffset = new Transform3d();

    public static final Set<Integer> hubIDs = Set.of
    (
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
    );

    public static final Set<Integer> towerIDs = Set.of
    (
      /* RED */
      15, 16,

      /* BLUE */
      31, 32
    );

    public static final Set<Integer> outpostIDs = Set.of
    (
      /* RED */
      13, 14,

      /* BLUE */
      29, 30
    );

    public static final Set<Integer> trenchIDs = Set.of
    (      /* RED */
      6, 7, // Scoring Side
      1, 12, // Non-Scoring Side

      /* BLUE */
      17, 28, // Scoring Side
      22, 23 // Non-Scoring Side
    );

    public static final Set<Integer> allIDs = Set.of();
    static
    {
      allIDs.addAll(trenchIDs);
      allIDs.addAll(outpostIDs);
      allIDs.addAll(towerIDs);
      allIDs.addAll(hubIDs);
    }

    

    /** Baseline 1 meter, 1 tag stddev for x and y, in meters */
    public static final double linearStdDevBaseline = 0.08;
    /** Baseline 1 meter, 1 tag stddev rotation, in radians */
    public static final double rotStdDevBaseline = 0.5;
    /** How many good MT1 readings to get before setting rotation and moving to MT2 */
    public static final int mt1CyclesNeeded = 10;
  }

  public static final class Interpolation 
  {
    public static final InterpolatingDoubleTreeMap shooterAltitudeHub = new InterpolatingDoubleTreeMap()
    {{
      put(0.0, 0.0);
      put(0.25, 0.25);
    }};

    public static final InterpolatingDoubleTreeMap shooterAltitudeLow = new InterpolatingDoubleTreeMap()
    {{
      put(0.0, 0.0);
      put(0.25, 0.25);
    }};

    public static final InterpolatingDoubleTreeMap turretPotAzimuth = new InterpolatingDoubleTreeMap()
    {{
      put(0.0, 0.0);
      put(0.25, 0.25);
    }};

    public static final InterpolatingDoubleTreeMap turretPotAltitude = new InterpolatingDoubleTreeMap()
    {{
      put(0.0, 0.0);
      put(0.25, 0.25);
    }}; 
  }

  public  static final class IndexerConstants 
  {
    public static final double speed = 0.5;
    
  }


  public static final class HopperConstants
  {
    public static final double spindexerPulseDelay = 0.25;
    public static final double spindexerSpeed = 0.5;
    public static final double intakeSpeed = 0.5; 

    
    public static final class ExtensionConstants 
    {
      // TODO actual ratios and gains
      public static final double extensionPlanetaryRatio = 1;
      public static final double extensionPinionTeeth = 1;
      public static final double extensionRackTeeth = 1;
      public static final double extensionRackRatio = extensionPinionTeeth / extensionRackTeeth;
      public static final double extensionRatio = extensionPlanetaryRatio * extensionRackRatio;

      public static final double maxRotations = 1;
      public static final double extensionJostleDelay = 0.25;

      private static final double gainS = 0.0;
      private static final double gainV = 0.0;
      private static final double gainA = 0.0;
      private static final double gainP = 0.0;
      private static final double gainI = 0.0;
      private static final double gainD = 0.0;

      public static final TalonFXConfiguration config = new TalonFXConfiguration();
      {
        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicAcceleration = 0;

        config.Slot0.kS = gainS;
        config.Slot0.kV = gainV;
        config.Slot0.kA = gainA;
        config.Slot0.kP = gainP;
        config.Slot0.kI = gainI;
        config.Slot0.kD = gainD;

        config.Slot1.kS = gainS;
        config.Slot1.kV = gainV;
        config.Slot1.kA = gainA;
        config.Slot1.kP = gainP;
        config.Slot1.kI = gainI;
        config.Slot1.kD = gainD;
      };
    }


  
  }   //TODO change maxrotaions 
  public static final class ClimberConstants {
    public static final double maxRotations = 1;

    public static final TalonFXConfiguration config = new TalonFXConfiguration();
  
  
    
  }

}