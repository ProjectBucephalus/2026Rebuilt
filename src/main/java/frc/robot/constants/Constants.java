package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.AllianceTranslation2d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constant values for mechanism geometry, motor configuration, input parameters, targets, etc. <p>
 * @Note For coordinate definitions, see Robot
 * <li> When possible, all units are Metres and Degrees (right-hand rotation)
 * <li> Gear ratios are defined by Teeth-Out/Teeth-In (except for pre-made gearboxes)
 * 
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
    public static final double maxThrottle = 0.8;
    /** Minimum robot speed when braking, relative to maximum uncapped speed */
    public static final double minThrottle = 0.2;
    /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
    public static final double maxRotThrottle = 0.8;
    /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
    public static final double minRotThrottle = 0.3;
    /** Maximum brake value when intake is running at full speed */
    public static final double brakeFromIntake = 0.4;
    /** How far a trigger must be pressed to be considered on, [0..1] */
    public static final double triggerThreshold = 0.8;
    /** Translation lineup tolerance, meters */
    public static final double lineupTolerance = 0.1;
    /** Rotation lineup tolerance, degrees */
    public static final double angleLineupTolerance = 4;

    /** Maximum robot speed to reduce power consumption, relative to maximum uncapped speed */
    public static final double powerSaveThrottle = 0.5;

    /** Attractor minimum angle tolerance, degrees */
    public static final double minAngleTolerance = 20;
    /** Attractor maximum angle tolerance, degrees */
    public static final double maxAngleTolerance = 60;

    public static final double manualIntakeExtensionScale  = 0.08;
    public static final double manualClimberExtensionScale = 0.05;
    public static final double manualShooterAzimuthAmount = 1.8;
    public static final double manualShooterDistanceAmount = 0.03;
    public static final double manualControlDeadband = 0.25;

    public static final double preShiftMargin = 1;
    public static final double postShiftMargin = 2;

    public static final double preShiftOutputMargin = 1;
    public static final double postShiftOutputMargin = 2;

    public static final double lastClimbChance = 10;

    /** Frequency for CAN device status signals that are needed, Hz */
    public static final double signalFrequency = 50;

    @SuppressWarnings("unchecked") // No way to make it work that doesn't give warning afaik
    /** First element is the default */
    public static final Pair<String, String>[] autoPresets = new Pair[]
    {
      new Pair<>("Depo & Climb", "WaitFor 3, Intake on, Follow l_depo, Intake agitate, WaitFor 1, Climb left"),
      new Pair<>("Drive Back", "Intake on, DriveBy -1 0"),
      new Pair<>("Right 1-cycle", "Follow r_trench_i2m, Intake on, Follow r_balls, Follow r_trench_m2a, Intake agitate, DriveTo 2 1.5, Climb right"),
      new Pair<>("Left 1-cycle", "Follow l_trench_i2m, Intake on, Follow l_balls, Follow l_trench_m2a, Intake agitate, DriveTo 2 6.5, Climb left"),
      new Pair<>
      (
        "Right 2-cycle", 
        "Follow r_trench_i2m, Intake on, Follow r_balls, Follow r_trench_m2a, Intake agitate, DriveTo 2 1.5, WaitFor 3, Intake reverse, Follow r_trench_a2m_r, Intake on, Follow r_balls_near, Follow r_trench_m2a, Intake agitate, Climb right"
      ),
      new Pair<>
      (
        "Left 2-cycle", 
        "Follow l_trench_i2m, Intake on, Follow l_balls, Follow l_trench_m2a, Intake agitate, DriveTo 2 6.5, WaitFor 3, Intake reverse, Follow l_trench_a2m_r, Intake on, Follow l_balls_near, Follow l_trench_m2a, Intake agitate, Climb left"
      ),
      new Pair<>("Blank", "")
    };
  }

  /** Geometry and tuning data for drivebase */
  public static final class SwerveConstants
  {
    /** Forward offset between the centre of the drivebase and Robot coordinate origin, metres */
    public static final double drivebaseOffset = 0.095;
    /** Offset from typical centre of rotation to centre of drivebase, metres Fore/Port */
    public static final Translation2d retractedCentreOffset = new Translation2d(-drivebaseOffset,0);
    /** Centre-centre distance between wheels port-stbd, metres */
    public static final double drivebaseWidth = 0.56;
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

    /* Auto Drive PID Values, Meters */
    public static final double driveKP = 2.25;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Auto Rotation PID Values, Degrees */
    public static final double rotationKP = 2.5;
    public static final double rotationKI = 0.0;
    public static final double rotationKD = 0.03;

    /* Swerve Limit Values */
    /** Mechanical maximum staright-line robot speed, Meters per Second */
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Mechanical maximum robot rotation rate, Radians per Second */
    public static final double maxAngularVelocity = 4;

    /** Radius from robot centre in metres where geofence is triggered for slow movements */
    public static final double robotRadiusInscribed = 0.42;
    /** Radius from robot centre in metres where geofence is triggered for fast movements */
    public static final double robotRadiusCircumscribed = 0.6;
    /** Radius enclosing robot when extended in metres where geofence is triggered for most movement */
    public static final double robotRadiusExpanded = 0.66;
    /** Speed threshold at which the robot changes between radii, m/s */
    public static final double robotSpeedThreshold = 1.5;
  }

  /** Geometry and tuning data for shooter systems */
  public static final class ShooterConstants
  {
    /** Target azimuth for Port shooter when climbing Right, degrees */
    public static final double towerAimPortAz   = -50;
    /** Target distance for Port shooter when climbing Right, metres */
    public static final double towerAimPortDist = 4;
    /** Target azimuth for Stbd shooter when climbing Left, degrees */
    public static final double towerAimStbdAz   = 60;
    /** Target distance for Stbd shooter when climbing Left, metres */
    public static final double towerAimStbdDist = 4;

    /** 
     * 2D offset from robot centre to port-side turret centre, metres fore/port, 
     * and rotation offset from robot-forward to turret-forward 
     */
    public static final Transform2d portShooterOffset = 
        new Transform2d(-(0.1635 + SwerveConstants.drivebaseOffset), 0.1815, Rotation2d.k180deg);
    /** 
     * 2D offset from robot centre to starboard-side turret centre, metres fore/port, 
     * and rotation offset from robot-forward to turret-forward 
     */
    public static final Transform2d stbdShooterOffset = 
        new Transform2d(-(0.1635 + SwerveConstants.drivebaseOffset), -0.1815, Rotation2d.k180deg);
    /** Distance either side of target for shooters to aim at to avoid balls coliding in flight, metres */
    public static final double targetPointOffset = 0.08;

    /** Expected delay between commanded and actual aim, seconds */
    public static final double mechanismLag = 0.15;
    /** Scale applied to acceleration for shot-leading pose-projection */
    public static final double accelLeadFactor = 0.8;
    /** Maximum Acceleration from drivebase at which shot leading is viable, metres per second^2 */
    public static final double leadingAccelLimit = 35;
    /** Maximum Jerk from drivebase at which shot leading is viable, metres per second^3 */
    public static final double leadingJerkLimit = 2000;
    /** Maximum drivebase speed to allow shooting when shooter-power-save mode is active, m/s */
    private static final double driveSpeedThreshold = 0.5;
    /** Maximum drivebase speed to allow shooting when shooter-power-save mode is active, squared to reduce calculation load, (m/s)^2 */
    public static final double driveSpeedSquareThreshold = Math.pow(driveSpeedThreshold, 2);

    public static final double minRange = 1.1;
    public static final double closeManualRange = 2;
    public static final double farManualRange = 4.5;
    public static final double maxPassRange = 7.0;

    /** Target pass point, blue origin (right side) */
    public static final AllianceTranslation2d passPoint = new AllianceTranslation2d(1.5, 2);

    /** Tuning data for flywheels */
    public static final class FlywheelConstants
    {
      private static final double motorPulley = 18;
      private static final double mainWheelPulley = 24;
      public static final double mainWheelBeltRatio = mainWheelPulley / motorPulley;

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

        flywheelConfig.Slot0.kS = 0.21;
        flywheelConfig.Slot0.kV = 0.162;
        flywheelConfig.Slot0.kA = 0.0;
        flywheelConfig.Slot0.kP = 0.09;
        flywheelConfig.Slot0.kI = 0.0;
        flywheelConfig.Slot0.kD = 0.0;

        flywheelConfig.MotionMagic.MotionMagicAcceleration = 250.0;
        flywheelConfig.MotionMagic.MotionMagicJerk = 1000.0;

        flywheelConfig.CurrentLimits.StatorCurrentLimit = 25;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      }

      /** Target flywheel speed when idle, mechanism rps */
      public static final double idleSpeed = 15;
      /** Allowed variation in flywheel speed for shooting, rps */
      public static final double flySpeedTolerance = 2;

      //simulation
      public static final double kGearRatio = 10.0;
      public static final double kMOI = 0.001; 
    }

    /** Geometry data for shooter hoods */
    public static final class HoodConstants 
    {
      /** Allowed variation in hood altitude when targeting, degrees */
      public static final double altTolerance = 1;
      /** Angle range of servo given input of [0..1], degrees anticlockwise */
      public static final double servoRange = 250;
      /** Range of motion of hood, degrees */
      public static final double hoodRange = 23;

      /** Offset for Zero position on port servo (servo degrees) */
      public static final double portHomeAngle = 11;
      /** Offset for Zero position on starboard servo (servo degrees) */
      public static final double stbdHomeAngle = 6;

      public static final double servoGear = 20;
      public static final double hoodGear = 193;
      public static final double hoodRatio = hoodGear / servoGear;
    }

    /** Geometry and tuning data for turret rings */
    public static final class TurretConstants
    {
      /** Maximum rotation either side of centre before reaching mechanical/cable limits, degrees */
      public static final double maxTurretAzimuth = 235;
      /** Angle range at end-of-travel to stop shooting and prepare to unwind, degrees */
      public static final double limitBufferZone = 10;
      /** Position to hold when idle, degrees */
      public static final double turretIdlePosition = 0;
      /** Target rotation rate when moving, rps */
      public static final double turretTurnSpeed = 2.3;
      /** Angle range of potentiometer giving output of [0..1], degrees */
      public static final double potRange = 3600;
      /** Angle offset to give 0 when turret is at centre, degrees */
      public static final double portPotOffset = -1790.00;
      /** Angle offset to give 0 when turret is at centre, degrees */
      public static final double stbdPotOffset = -1805.00;

      private static final double planetaryRatio = 13.03; // MaxPlanetary gearbox marked 4:1 is actually 3.6:1, 5:1 is actually 5.2:1
      private static final double driveGear = 15;
      private static final double ringGear = 90;
      public static final double azimuthGearRatio = ringGear / driveGear;
      public static final double azimuthPotRatio = -azimuthGearRatio;
      public static final double azimuthMotorRatio = azimuthGearRatio * planetaryRatio;

      /** Allowed variation in turret azimuth when targeting, degrees */
      public static final double azimuthTolerance = 6;

      /** Maximum absolute rotation rate of the turret in field-space to be considered safe to shoot, rps */
      public static final double maxRPS = 1;

      /** Maximum expected value from potentiometer, beyond which indicates error, sensor degrees */
      public static final double potSafeLimit = 265 * azimuthGearRatio;
      /** Minimum change in azimuth before recalibrating, sensor degrees */
      public static final double calibrationAngleLimit = 5 * Math.abs(azimuthGearRatio);
      /** Maximum robot-relative rotation rate to calibrate turret, rotations per second */
      public static final double calibrationSpeedLimit = 0.02;
      
      public static final TalonFXSConfiguration turretConfig = new TalonFXSConfiguration();
      static 
      {
        turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretConfig.ExternalFeedback.SensorToMechanismRatio = azimuthMotorRatio;

        turretConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        
        turretConfig.Slot0.kS = 0.5;
        turretConfig.Slot0.kV = 5.4;
        turretConfig.Slot0.kA = 0.0;
        turretConfig.Slot0.kP = 5.29;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0.01;

        turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretTurnSpeed;
        turretConfig.MotionMagic.MotionMagicAcceleration = turretTurnSpeed * 5;

        turretConfig.CurrentLimits.StatorCurrentLimit = 23;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      }
    }

    /** Tuning data for indexer */
    public static final class IndexerConstants 
    {
      public static final double indexerSpeed = 50;
      public static final double indexerMinSpeed = 25;
      public static final double indexerReverseSpeed = -25;

      private static final double gearboxRatio = 1;
      public static final double flywheelSpeedRatio = 2.9;

      public static final TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
      static
      {
        indexerConfig.Feedback.SensorToMechanismRatio = gearboxRatio;

        indexerConfig.Slot0.kS = 0.25;
        indexerConfig.Slot0.kV = 0.125;
        indexerConfig.Slot0.kA = 0.0;
        indexerConfig.Slot0.kP = 0.16;
        indexerConfig.Slot0.kI = 0.01;
        indexerConfig.Slot0.kD = 0.0;

        indexerConfig.MotionMagic.MotionMagicAcceleration = 80.0;

        indexerConfig.CurrentLimits.StatorCurrentLimit = 30;
      }
    }
  }

  /** Geometry, tag, and tuning data for Vision system */
  public static final class VisionConstants
  {
    /** X/Y offset of camera in turret space, metres aft/stbd */
    public static final Transform2d flatCameraToTurret = new Transform2d(-0.156, 0, Rotation2d.kZero);
    private static final Translation2d baseTurretToCamera = new Translation2d(0.156, 0.195);
    /** Pitch of camera in turret space, degrees */
    private static final double turretPitch = -12.9;
    /** X/Z offset of camera in camera space, metres fore/up */
    private static final Translation2d turretToCamera = baseTurretToCamera.rotateBy(Rotation2d.fromDegrees(turretPitch));

    /** 3D offset from centre of rotation of turret at floor level to centre of camera lens, metres fore*2/port/down, degrees roll/pitch/yaw */
    public static final Transform3d portLimelightOffset = new Transform3d(turretToCamera.getX(), 0, turretToCamera.getY(), new Rotation3d(0, Math.toRadians(turretPitch), 0));
    /** 3D offset from centre of rotation of turret at floor level to centre of camera lens, metres fore*2/port/down, degrees roll/pitch/yaw */
    public static final Transform3d stbdLimelightOffset = new Transform3d(turretToCamera.getX(), 0, turretToCamera.getY(), new Rotation3d(0, Math.toRadians(turretPitch), 0));
    /** Maximum time between vision estimates before switching to odometry only, seconds */
    public static final double visionFrequencyThreshold = 2;
    /** How many seconds into the past we store turret azimuth readings */
    public static final double azimuthBufLength = 10;

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
    }

    /** Baseline 1 meter, 1 tag stddev for x and y, meters */
    public static final double linearStdDevBaseline = 0.2;
    /** Baseline 1 meter, 1 tag stddev rotation, radians */
    public static final double rotStdDevBaseline = Math.toRadians(40);
  }

  /** Interpolation tables for converting measured input to calibrated output */
  public static final class Interpolation 
  {
    /** Distance to Altitude conversion for shooting into the elevated Hub */
    public static final InterpolatingDoubleTreeMap shooterAltitudeHub = new InterpolatingDoubleTreeMap();
    static 
    {
      shooterAltitudeHub.put(0.0, 0.0);
      shooterAltitudeHub.put(1.0, 0.0);
      shooterAltitudeHub.put(1.1, 1.0);
      shooterAltitudeHub.put(1.5, 4.0);
      shooterAltitudeHub.put(2.0, 8.0);
      shooterAltitudeHub.put(2.5, 11.0);
      shooterAltitudeHub.put(3.0, 13.0);
      shooterAltitudeHub.put(3.5, 16.0);
      shooterAltitudeHub.put(4.0, 19.0);
      shooterAltitudeHub.put(4.5, 22.0);
      shooterAltitudeHub.put(5.0, 23.0);
      shooterAltitudeHub.put(5.5, 23.0);
    }

    /** Distance to Speed conversion for shooting into the elevated Hub */
    public static final InterpolatingDoubleTreeMap flywheelSpeedHub = new InterpolatingDoubleTreeMap();
    static
    {
      flywheelSpeedHub.put(0.0, 0.0);
      flywheelSpeedHub.put(0.1, 0.0);
      flywheelSpeedHub.put(0.9, 45.0); // below min range
      flywheelSpeedHub.put(1.1, 45.0);
      flywheelSpeedHub.put(1.5, 46.5);
      flywheelSpeedHub.put(2.0, 48.0);
      flywheelSpeedHub.put(2.5, 49.0);
      flywheelSpeedHub.put(3.0, 50.0);
      flywheelSpeedHub.put(3.5, 52.0);
      flywheelSpeedHub.put(4.0, 53.0); 
      flywheelSpeedHub.put(4.5, 56.0);
      flywheelSpeedHub.put(5.0, 59.0);
      flywheelSpeedHub.put(5.5, 61.0);
      flywheelSpeedHub.put(6.0, 62.0);
      flywheelSpeedHub.put(6.5, 62.0);
    }

    /** Distance to Altitude conversion for shooting to a point on the field */
    public static final InterpolatingDoubleTreeMap shooterAltitudeLow = new InterpolatingDoubleTreeMap();
    static
    {
      shooterAltitudeLow.put(0.5, 23.0);
      shooterAltitudeLow.put(7.5, 23.0);

    }

    /** Distance to Speed conversion for shooting to a point on the field */
    public static final InterpolatingDoubleTreeMap flywheelSpeedLow = new InterpolatingDoubleTreeMap();
    static
    {
      flywheelSpeedLow.put(0.5, 15.0);
      flywheelSpeedLow.put(1.5, 22.0);
      flywheelSpeedLow.put(2.5, 33.0);
      flywheelSpeedLow.put(3.5, 39.0);
      flywheelSpeedLow.put(4.5, 46.0);
      flywheelSpeedLow.put(5.5, 51.0);
      flywheelSpeedLow.put(6.5, 58.0);
      flywheelSpeedLow.put(7.0, 60.0);
      flywheelSpeedLow.put(7.5, 60.0);
    }

    /** Distance to Time-of-Flight for Shoot-on-the-Move */
    public static final InterpolatingDoubleTreeMap shotTime = new InterpolatingDoubleTreeMap();
    static
    {
      shotTime.put(1.0, 1.17);
      shotTime.put(1.5, 1.21);
      shotTime.put(2.0, 1.23);
      shotTime.put(2.5, 1.24);
      shotTime.put(3.0, 1.25);
      shotTime.put(3.5, 1.29);
      shotTime.put(4.0, 1.27);
      shotTime.put(4.5, 1.31);
      shotTime.put(5.0, 1.37);
      shotTime.put(5.5, 1.43);
      shotTime.put(6.0, 1.45);
    }
  }

  /** Geometry and tuning data for intake system */
  public static final class IntakeConstants
  {
    public static final class RollerConstants 
    {
      /** Default speed of intake when running, rps */
      public static final double intakeMaxSpeed = 50;
      /** Reduced speed for intake when reversing or to reduce power consumption, rps */
      public static final double intakeMinSpeed = 30;

      /** Intake speed at which robot throttle starts being applied, rps */
      public static final double brakeSpeedStart = 10;
      /** Intake speed at which maximum robot throttle is applied, rps above throttleStart */
      public static final double brakeSpeedRange = 40 - brakeSpeedStart;
      /** Maximum brake value */
      public static final double intakeBrake = 0.5;
      public static final double maxSpeedThreshold = 2.5;
      
      public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
      static
      {
        intakeConfig.Feedback.SensorToMechanismRatio = 1.0;

        intakeConfig.Slot0.kS = 0.265;
        intakeConfig.Slot0.kV = 0.1;
        intakeConfig.Slot0.kI = 0.01;

        intakeConfig.Slot0.kP = 0.15;

        intakeConfig.MotionMagic.MotionMagicAcceleration = 200.0;

        intakeConfig.CurrentLimits.StatorCurrentLimit = 55;
      }
    }

    /** Geometry and tuning data of intake extension */
    public static final class ExtensionConstants 
    {
      private static final double extensionPlanetaryRatio = 9;

      private static final double extensionInGear = 20;
      private static final double extensionOutGear = 40;
      private static final double extensionGearRatio = extensionOutGear / extensionInGear;

      private static final double extensionInPulley = 15;
      private static final double extensionOutPulley = 30;
      private static final double extensionChainRatio = extensionOutPulley / extensionInPulley;

      public static final double minRotations = -0.31;
      public static final double maxRotations = 0.0;
      public static final double jostleRotations = -0.05;
      public static final double bumpSafeRotations = -0.05;

      public static final TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
      static
      {
        extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        extensionConfig.Feedback.FeedbackRemoteSensorID = IDConstants.extensionEncoderCAN;
        extensionConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        extensionConfig.Feedback.RotorToSensorRatio = extensionPlanetaryRatio * extensionGearRatio;
        extensionConfig.Feedback.SensorToMechanismRatio = extensionChainRatio;

        extensionConfig.Slot0.kS = 0.125;
        extensionConfig.Slot0.kP = 55.0;
        extensionConfig.Slot0.kI = 0.0;
        extensionConfig.Slot0.kD = 0.0;
        extensionConfig.Slot0.kG = 0.375;

        extensionConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
        extensionConfig.MotionMagic.MotionMagicAcceleration = 1.5;

        extensionConfig.CurrentLimits.StatorCurrentLimit = 35;
        extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      }
    }
  }   

  /** Geometry and tuning data of climber system */
  public static final class ClimberConstants 
  {
    /** meters */
    public static final double minPosition  = 0.015;
    /** meters */
    public static final double maxPosition  = 0.23;
    /** Position set when climber calibrates, meters */
    public static final double homePosition = 0.02;
    /** Alternate between `maxPosition` and `max - offset` for final approach to tower, metres */
    public static final double wiggleOffset = 0.009;
    /** Position for full climb, meters */
    public static final double climbPosition = 0.04;

    /** Delay between movements when approaching tower, seconds */
    public static final double wiggleWait = 0.05;

    private static final double planetaryRatio = 25;
    private static final double motorPulley = 12;
    private static final double winchPulley = 15;
    private static final double winchChainRatio = winchPulley / motorPulley;

    /** meters */
    public static final double metersPerRotation = 0.061;

    /** rotations per second, mechanical maximum 3.2 */
    private static final double cruiseVelocity = 3.2;
    /** rotations per second */
    private static final double cruiseVelocityUncalibrated = 0.5;

    public static final TalonFXConfiguration climberConfig = new TalonFXConfiguration(); 
    static 
    {
      climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      climberConfig.Feedback.SensorToMechanismRatio = winchChainRatio * planetaryRatio;

      climberConfig.Slot0.kS = 0.2;
      climberConfig.Slot0.kP = 55.0;
      climberConfig.Slot0.kI = 0.0;
      climberConfig.Slot0.kD = 0.0;
      
      climberConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity; // Mechanical maximum 3.2
      climberConfig.MotionMagic.MotionMagicAcceleration = 25 * cruiseVelocity;
      
      climberConfig.CustomParams.CustomParam0 = (int) (cruiseVelocityUncalibrated * 100); // Cruise velocity to use when not calibrated, 1/100 mechanism rotations per second
    }
  }
}
