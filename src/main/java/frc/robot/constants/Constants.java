package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/*
 * NOTES:
 * For coordinate definitions, see Robot
 * When possible, all units are Metres and Degrees
 * Gear ratios are defined by Teeth-Out/Teeth-In (except for pre-made gearboxes)
 */

/**
 * Constant values for mechanism geometry, motor configuration, input parameters, targets, etc.
 * @author 5985
 */
public final class Constants 
{
  public static final class RumblerConstants 
  {
    public static final double driverDefault = 0.1;
    public static final double operatorDefault = 0.1;
  }

  /** Values for controller input and general driving behaviours */
  public static final class ControlConstants
  {
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
    /** Translation lineup tolerance, meters */
    public static final double lineupTolerance = 0.05;
    /** Rotation lineup tolerance, degrees */
    public static final double angleLineupTolerance = 3;
  }

  /** Geometry and tuning data for drivebase */
  public static final class SwerveConstants
  {
    /** Forward offset between the centre of the drivebase and Robot coordinate origin, metres */
    public static final double drivebaseOffset = 0.095;
    /** Offset from typical centre of rotation to centre of drivebase, metres Fore/Port */
    public static final Translation2d retractedCentreOffset = new Translation2d(-drivebaseOffset,0);
    /** Centre-centre distance between wheels port-stbd, metres */
    public static final double drivebaseWidth = 0.485;
    /** Centre-centre distance between wheels fore-aft, metres */
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

    /* Drive PID Values */
    public static final double driveKP = 2.5;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.12;

    /* Rotation Control PID Values */
    public static final double rotationKP = 6;
    public static final double rotationKI = 0;
    public static final double rotationKD = 0;

    /* Swerve Limit Values */
    /** Mechanical maximum staright-line robot speed, Meters per Second */
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Mechanical maximum robot rotation rate, Radians per Second */
    public static final double maxAngularVelocity = 4;
  }

  /** Geometry and tuning data for shooter systems */
  public static final class ShooterConstants
  {
    /** 2D offset from robot centre to port-side turret centre of rotation, metres fore/port, and rotation offset from robot-forward to turret-forward */
    public static final Transform2d portShooterOffset = new Transform2d(-(0.1635 + SwerveConstants.drivebaseOffset), 0.1815, Rotation2d.kZero);
    /** 2D offset from robot centre to starboard-side turret centre of rotation, metres fore/port, and rotation offset from robot-forward to turret-forward */
    public static final Transform2d stbdShooterOffset = new Transform2d(-(0.1635 + SwerveConstants.drivebaseOffset), -0.1815, Rotation2d.kZero);

    /** Tuning data for flywheels */
    public static final class FlywheelConstants
    {
      private static final double motorPulley = 24;
      private static final double mainWheelPulley = 18;
      private static final double mainWheelBeltRatio = mainWheelPulley / motorPulley;

      /*
      * To tune flywheel:
      *    Find voltage KS required to overcome static friction
      *    Run with voltage at maximum safe limit, record voltage and RPS
      *    Set voltage KV as voltage/RPS
      *    Once KV is tuned, use KP for additional gain as needed
      */
      public static final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration(); 
      static
      {
        flywheelConfig.Feedback.SensorToMechanismRatio = mainWheelBeltRatio;

        flywheelConfig.Slot0.kS = 0.2;
        flywheelConfig.Slot0.kV = 0.08;
        flywheelConfig.Slot0.kA = 0.0;
        flywheelConfig.Slot0.kP = 0.0;
        flywheelConfig.Slot0.kI = 0.0;
        flywheelConfig.Slot0.kD = 0.0;

        flywheelConfig.MotionMagic.MotionMagicAcceleration = 100.0;
        flywheelConfig.MotionMagic.MotionMagicJerk = 500.0;
      }

      /** Target flywheel speed when idle, mechanism rps */
      public static final double idleSpeed = 30;
      /** Target flywheel speed for shooting, mechanism rps */
      public static final double revSpeed = 100;
      /** Allowed variation in flywheel speed for shooting, rps */
      public static final double flySpeedTolerance = 5;

      //simulation
      public static final double kGearRatio = 10.0;
      public static final double kMOI = 0.001; 
    }

    /** Geometry data for shooter hoods */
    public static final class HoodConstants 
    {
      /** Allowed variation in hood altitude when targeting, degrees */
      public static final double altTolerance = 1;
      /** Angle range of servo given input of [0..1], degrees */
      public static final double servoRange = 300;
      /** Range of motion of hood, degrees */
      public static final double hoodRange = 25;

      public static final double servoGear = 20;
      public static final double hoodGear = 193;
      public static final double hoodRatio = hoodGear / servoGear;
    }

    /** Geometry and tuning data for turret rings */
    public static final class TurretConstants
    {
      /** Maximum rotation either side of centre before reaching mechanical/cable limits, degrees */
      public static final double maxTurretAzimuth = 270;
      /** Angle range at end-of-travel to stop shooting and prepare to unwind, degrees */
      public static final double limitBufferZone = 10;
      /** Position to hold when idle, degrees */
      public static final double turretIdlePosition = 0;
      /** Target rotation rate when moving, rps */
      public static final double turretTurnSpeed = 0.25;
      /** Angle range of potentiometer giving output of [0..1], degrees */
      public static final double potRange = 3600;
      /** Angle offset to give 0 when turret is at centre, degrees */
      public static final double potPortOffset = -1800;
      /** Angle offset to give 0 when turret is at centre, degrees */
      public static final double potStbdOffset = -1800;

      private static final double planetaryRatio = 12;
      private static final double driveGear = 20;
      private static final double ringGear = 90;
      public static final double azimuthGearRatio = ringGear / driveGear;

      /** Allowed variation in turret azimuth when targeting, degrees */
      public static final double azimuthTolerance = 3;

      /** Maximum absolute rotation rate of the turret in field-space to be considered safe to shoot, rps */
      public static final double maxRPS = 1;
      
      public static final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
      static 
      {
        turretConfig.Feedback.SensorToMechanismRatio = azimuthGearRatio * planetaryRatio;

        turretConfig.Slot0.kS = 0.0;
        turretConfig.Slot0.kV = 0.0;
        turretConfig.Slot0.kA = 0.0;
        turretConfig.Slot0.kP = 10.0;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0.0;

        turretConfig.MotionMagic.MotionMagicAcceleration = turretTurnSpeed * 5;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretTurnSpeed;
      }
    }
  }

  /** Geometry, tag, and tuning data for Vision system */
  public static final class VisionConstants
  {
    /** 3D offset from centre of rotation of turret at floor level to centre of camera lens, metres fore/port/up, degrees roll/pitch/yaw */
    public static final Transform3d portLimelightOffset = new Transform3d(0, 0, 0, new Rotation3d(0, -15, 0));
    /** 3D offset from centre of rotation of turret at floor level to centre of camera lens, metres fore/port/up, degrees roll/pitch/yaw */
    public static final Transform3d stbdLimelightOffset = new Transform3d(0, 0, 0, new Rotation3d(0, -15, 0));
    /** Maximum time between vision estimates before switching to odometry only, seconds */
    public static final double visionFrequencyThreshold = 10;

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
    (
      /* RED */
      6, 7, // Scoring Side
      1, 12, // Non-Scoring Side

      /* BLUE */
      17, 28, // Scoring Side
      22, 23 // Non-Scoring Side
    );

    public static final Set<Integer> allIDs = new HashSet<>(32);
    static
    {
      allIDs.addAll(trenchIDs);
      allIDs.addAll(outpostIDs);
      allIDs.addAll(towerIDs);
      allIDs.addAll(hubIDs);
    };

    /** Baseline 1 meter, 1 tag stddev for x and y, meters */
    public static final double linearStdDevBaseline = 0.06;
    /** Baseline 1 meter, 1 tag stddev rotation, radians */
    public static final double rotStdDevBaseline = Math.toRadians(8);
  }

  /** Interpolation tables for converting measured input to calibrated output */
  public static final class Interpolation 
  {
    /** Distance to Altitude conversion for shooting into the elevated Hub */
    public static final InterpolatingDoubleTreeMap shooterAltitudeHub = new InterpolatingDoubleTreeMap()
    {{
      put(0.0, 0.0);
      put(0.25, 0.25);
    }};
    
    /** Distance to Altitude conversion for shooting to a point on the field */
    public static final InterpolatingDoubleTreeMap shooterAltitudeLow = new InterpolatingDoubleTreeMap()
    {{
      put(0.0, 0.0);
      put(0.25, 0.25);
    }};
  }

  /** Tuning data for feeder */
  public  static final class FeederConstants 
  {
    public static final double feederSpeed = 0.5;

    private static final double gearboxRatio = 1;
    private static final double lowerRollerPulley = 24;
    private static final double upperRollerPuller = 18;
    private static final double rollerBeltRatio = upperRollerPuller / lowerRollerPulley;
    private static final double motorToUpperRatio = rollerBeltRatio * gearboxRatio;

    public static final TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    static
    {
      feederConfig.Feedback.SensorToMechanismRatio = motorToUpperRatio;
    }
  }

  /** Geometry and tuning data for hopper system */
  public static final class HopperConstants
  {
    public static final class SpindexerConstants 
    {
      /** Duration and interval of spindexer pulses when agitating, seconds */
      public static final double spindexerPulseDelay = 0.25;
      /** Default speed of spindexer when running, [-1..1] */
      public static final double spindexerSpeed = 0.5;

      private static final double motorPulley = 24;
      private static final double spindexerPulley = 30;
      private static final double spindexerBeltRatio = spindexerPulley / motorPulley;
      
      public static final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
      static
      {
        spindexerConfig.Feedback.SensorToMechanismRatio = spindexerBeltRatio;
      }
    }

    public static final class IntakeConstants 
    {
      /** Default speed of intake when running, [-1..1] */
      public static final double intakeSpeed = 0.5;
      
      public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
      static
      {
        intakeConfig.Feedback.SensorToMechanismRatio = 1.0;
      }
    }

    /** Geometry and tuning data of intake/hopper extension */
    public static final class ExtensionConstants 
    {
      // TODO actual ratios and gains
      public static final double extensionPlanetaryRatio = 1;
      public static final double extensionPinionTeeth = 1;
      public static final double extensionRackTeeth = 1;
      public static final double extensionRackRatio = extensionPinionTeeth / extensionRackTeeth;
      public static final double extensionRatio = extensionPlanetaryRatio * extensionRackRatio;

      public static final double maxRotations = 1;

      /** Duration and interval of retraction/extension pulses when agitating, seconds */
      public static final double extensionJostleDelay = 0.25;

      public static final TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
      static
      {
        extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        extensionConfig.MotionMagic.MotionMagicAcceleration = 0;

        extensionConfig.Slot0.kS = 0.0;
        extensionConfig.Slot0.kV = 0.0;
        extensionConfig.Slot0.kA = 0.0;
        extensionConfig.Slot0.kP = 0.0;
        extensionConfig.Slot0.kI = 0.0;
        extensionConfig.Slot0.kD = 0.0;

        extensionConfig.Slot1.kS = 0.0;
        extensionConfig.Slot1.kV = 0.0;
        extensionConfig.Slot1.kA = 0.0;
        extensionConfig.Slot1.kP = 0.0;
        extensionConfig.Slot1.kI = 0.0;
        extensionConfig.Slot1.kD = 0.0;
      };
    }
  }   

  /** Geometry and tuning data of climber system */
  public static final class ClimberConstants 
  {
    /** meters */
    public static final double maxPosition = 0.478;

    private static final double planetaryRatio = 25;
    private static final double motorPulley = 12;
    private static final double winchPulley = 15;
    private static final double winchChainRatio = winchPulley / motorPulley;

    /** meters */
    private static final double winchDiameter = 0.029;
    private static final double cordDiameter = 0.006;
    public static final double metersPerRotation = (winchDiameter + cordDiameter) * Math.PI;

    public static final TalonFXConfiguration climberConfig = new TalonFXConfiguration(); 
    static 
    {
      climberConfig.Feedback.SensorToMechanismRatio = winchChainRatio * planetaryRatio;
    }
  }
}