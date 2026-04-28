package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ShooterConstants.*;
import frc.robot.constants.Constants.Interpolation;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.constants.IDConstants.ShooterIDs;
import frc.robot.subsystems.generic.VelocityMotor;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

/**
 * Turreted Shooter master-system, internally creates and manages associated subsystems
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Shooter extends SubsystemBase 
{  
  @Logged
  private final Flywheels flywheels; 
  @Logged
  private final Turret turret;
  @Logged
  private final Hood hood;
  @Logged
  private final VelocityMotor indexer;
  
  private final Transform2d shooterOffset;
  private final Translation2d baseTargetOffset;

  private final String ntId;

  private final Supplier<SwerveDriveState> swerveStateSup;
  private SwerveDriveState swerveState;

  private Status shootStatus;

  private Pose2d shooterPose;

  private Translation2d lastPosition = Translation2d.kZero;
  private Translation2d lastVelocity = Translation2d.kZero;
  private Translation2d lastAcceleration = Translation2d.kZero;
  @Logged
  private double accel;
  @Logged
  private double jerk;
  private double timeOfFlight = 0;

  /** Current active target for the shooter */
  @Logged
  public final Target target = new Target(TargetState.Manual);

  /**
   * Creates Turreted Shooter master-system, internally creates and manages associated subsystems
   * @param swerveStateSup Supplier for drivebase state to access pose and motion values
   * @param robotToShooter Offset from robot-centre to turret-centre, including rotation
   * @param flywheelLeaderCAN CAN-ID of primary shooter motor
   * @param flywheelFollowerCAN CAN-ID of secondary shooter motor, set to follow first
   * @param turretCAN CAN-ID of turret azimuth motor
   * @param azimuthIO AIO-ID of azimuth potentiometer
   * @param azimuthOffset Potentiometer reading for centre of rotation
   * @param hoodPWM PWM-ID of hood altitude servo
   * @param hoodIO AIO-ID of hood feedback sensor
   * @param invertedHood Inverts the range and direction of motion of the hood servo
   * @param activeSup Supplier for when the shooter should be ready to shoot
   */
  public Shooter
  (
    Supplier<SwerveDriveState> swerveStateSup,
    Transform2d robotToShooter,
    ShooterIDs idBlock,
    double azimuthOffset,
    boolean invertedHood,
    double hoodHomeAngle,
    boolean invertedFeeder
  ) 
  {
    this.swerveStateSup = swerveStateSup;
    this.swerveState = swerveStateSup.get();
    this.shooterOffset = robotToShooter;
    shooterPose = Pose2d.kZero.plus(shooterOffset);

    baseTargetOffset = new Translation2d(0, Math.copySign(ShooterConstants.targetPointOffset, robotToShooter.getY()));
    ntId = idBlock.ntID();

    flywheels = new Flywheels(idBlock.flywheelLeadCAN(), idBlock.flywheelFollowCAN(), target);
    turret = new Turret(idBlock.azimuthCAN(), idBlock.azimuthAIO(), azimuthOffset, target);
    hood = new Hood(idBlock.altitudePWM(), invertedHood, hoodHomeAngle, target);
    
    var indexerDir = invertedFeeder ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    IndexerConstants.indexerConfig.MotorOutput.Inverted = indexerDir;
    indexer = new VelocityMotor(idBlock.indexerCAN(), IndexerConstants.indexerConfig);

    target.azimuth = turret.getAzimuth();
  }

  public Command adjustDistanceCmd(DoubleSupplier shiftSup)
  {
    return Commands.run(() -> 
    {
      target.distance += shiftSup.getAsDouble();
      target.distance = Math.max(target.distance, 0);
      target.speed = Interpolation.flywheelSpeedHub.get(target.distance);
      target.altitude = Interpolation.shooterAltitudeHub.get(target.distance);
    }).withName("Manual Distance");
  }

  public void setDistance(double distance)
  {
    target.distance = Math.max(distance, 0);
    target.speed = Interpolation.flywheelSpeedHub.get(target.distance);
    target.altitude = Interpolation.shooterAltitudeHub.get(target.distance);
  }

  public Command adjustAzimuthCmd(DoubleSupplier shiftSup)
    {return Commands.run(() -> target.azimuth += shiftSup.getAsDouble()).withName("Manual Azimuth");}

  /** @return Current robot-relative azimuth of the turret, degrees */
  public double getAzimuth()
    {return turret.getAzimuth();}

  /** @return A pair consisting of the timestamp of the last azimuth reading (in current seconds), and the reading itself (in degrees) */
  public Pair<Double, Double> getAzimuthTimestamped()
    {return turret.getAzimuthTimestamped();}

  /** @return Current speed of the flywheels (RPS of the main flywheel) */
  public double getSpeed()
    {return flywheels.getSpeed();}

  /**
   * Checks whether we are in a valid state to be shooting. This requires that:
   * <ul>
   * <li> The flywheels have been set to active
   * <li> The turret is ready, as per {@link Turret#readyToShoot()}
   * <li> The flywheels are at target speed, as per {@link Flywheels#atSpeed()}
   * <li> The distance to the target is at least {@link ShooterConstants#minRange minRange}
   * <li> The robot is not within a trench
   * <li> The robot is not within the "shadow" of either tower
   * </ul>
   * 
   * @return True if all above conditions are true
   */
  @Logged
  public boolean shootReady()
    {return shootStatus == Status.Fire || shootStatus == Status.AwaitingInput;}

  /**
   * Checks what conditions required for shooting are currently met
   * 
   * @return Status
   */
  @Logged
  public Status shootStatus()
    {return shootStatus;}

  private void updateStatus()
  {
    if (target.state == TargetState.Vision)
      shootStatus = Status.Vision;
    else if 
    (
      !target.flywheelsActive 
      || target.disabled
      || (PBDash.IO_POWER_SHOOT.get() && lastVelocity.getSquaredNorm() > ShooterConstants.driveSpeedSquareThreshold)
    )
      shootStatus = Status.Idling;
    else if 
    (
      target.distance <= ShooterConstants.minRange 
      || GeoFencing.obstacleBlue.checkPosition(shooterPose.getTranslation())
      || GeoFencing.obstacleRed.checkPosition(shooterPose.getTranslation())
      || GeoFencing.towerShadowBlue.checkPosition(shooterPose.getTranslation())
      || GeoFencing.towerShadowRed.checkPosition(shooterPose.getTranslation())
    )
      shootStatus = Status.BadLocation;
    else if (!turret.readyToShoot(swerveState.Speeds))
      shootStatus = Status.Aiming;
    else if (!flywheels.atSpeed() || (target.state != TargetState.Manual && jerk >= PBDash.TUNE_JERK_LIMIT.get()))
      shootStatus = Status.Revving;
    else if (indexer.getSpeed() <= 5)
      shootStatus = Status.AwaitingInput;
    else 
      shootStatus = Status.Fire;
  }

  public Command runIndexerCmd()
    {return indexer.runCmd(() -> Math.max(getSpeed(), IndexerConstants.indexerMinSpeed));}

  public Command runIndexerCmd(DoubleSupplier speedSup)
    {return indexer.runCmd(speedSup);}

  public Command stopIndexerCmd()
    {return indexer.runCmd(() -> 0.0);}

  /** Sets the flywheels to rev up to target speed */
  public void revFlywheels() {target.flywheelsActive = true;}
  /** Sets the flywheels to idle speed */
  public void idleFlywheels() {target.flywheelsActive = false;}

  public Command runFlywheelsCmd() {return Commands.startEnd(this::revFlywheels, this::idleFlywheels);}

  public boolean potValid() {return turret.potValid();}

  private void telemetrise()
  {
    var rotatedPose = shooterPose
      .plus(new Transform2d(Translation2d.kZero, Rotation2d.fromDegrees(turret.getAzimuth())));
    PBDash.putFieldPath(ntId + " Pose", rotatedPose, rotatedPose.transformBy(new Transform2d(flywheels.getSpeed() / 60, 0, Rotation2d.kZero)));

    var targetPoint = target.state == TargetState.Hub ? FieldUtils.getAllianceHubCentre() : target.point;
    PBDash.putFieldObject(ntId + "Target", new Pose2d(targetPoint.plus(target.offset), Rotation2d.kZero));
  }

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
  {
    return flywheels.devicesValid()
        && turret.devicesValid()
        && indexer.devicesValid();
  }

  /** Calibrates the turret to match the potentiometer */
  public void calibrate()
    {turret.calibrate(true);}

  @Override
  public void periodic()
  {
    swerveState = swerveStateSup.get();
    shooterPose = swerveState.Pose.plus(shooterOffset);
    target.shooterPosition = shooterPose.getTranslation();

    // Calculate the instantaneous velocity and acceleration of the shooter
    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerveState.Speeds, swerveState.Pose.getRotation());
    Translation2d velocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    Translation2d acceleration = velocity.minus(lastVelocity).times(50);
    accel = acceleration.getNorm();
    jerk = acceleration.minus(lastAcceleration).getNorm() * 50;

    // Store pose, velocity, and acceleration to be used next cycle
    //lastPosition = shooterPose.getTranslation(); // Shooter pose has too much noise, so currently using speed from drivebase
    lastVelocity = velocity;
    lastAcceleration = acceleration;

    // Find distance to current target for calculating leading shots
    double distance = switch (target.state) 
    {
      case Point -> target.point.minus(target.shooterPosition).getNorm();
      // aim at our alliance's hub
      case Hub -> FieldUtils.getAllianceHubCentre().minus(target.shooterPosition).getNorm();
      default -> target.distance;
    };

    if (target.state != TargetState.Manual && target.state != TargetState.Vision)
    {
      // Base target offset to reduce collisions
      target.offset = baseTargetOffset
          .rotateBy(swerveState.Pose.getRotation().unaryMinus());
      
      // If acceleration is stable, calculate shot leading
      if (jerk < PBDash.TUNE_JERK_LIMIT.get())
      { 
        double mechLag = PBDash.TUNE_MECH_LAG.get(); 

        // Projecting pose based on velocity and acceleration
        shooterPose = new Pose2d(
          shooterPose.getTranslation()
            .plus(velocity.plus(acceleration.times(mechLag * PBDash.TUNE_LEAD_FACTOR.get())).times(mechLag)), 
          shooterPose.getRotation());

        Translation2d targetPoint = switch (target.state) 
        {
          case Point -> target.point;
          // aim at our alliance's hub
          case Hub -> FieldUtils.getAllianceHubCentre();
          default -> Translation2d.kZero;
        };

        // Calculate the component of the velocity that is towards the target
        double motionNormal = (((targetPoint.getX() - shooterPose.getX()) * velocity.getX()) + ((targetPoint.getY() - shooterPose.getY()) * velocity.getY())) / distance; 
        double normalFactor = motionNormal / velocity.getNorm();

        // Multiply ToF from last cycle by velocity towards target to give the change in distance from shot leading
        // Use this new distance to calculate the new ToF
        timeOfFlight = Interpolation.shotTime.get(distance + (timeOfFlight * normalFactor * motionNormal));

        // Calculate target offset to avoid balls from each shooter colliding before reaching target
        // accounting for turret velocity and acceleration
        target.offset = target.offset
            .minus(velocity
            .plus(acceleration.times(PBDash.TUNE_LEAD_FACTOR.get() * mechLag * mechLag))
            .times(timeOfFlight)
            );

      }
      // Find distance to current target for calculating leading shots
      target.distance = switch (target.state) 
      {
        case Point -> target.point.plus(target.offset).minus(target.shooterPosition).getNorm();
        // aim at our alliance's hub
        case Hub -> FieldUtils.getAllianceHubCentre().plus(target.offset).minus(target.shooterPosition).getNorm();
        default -> target.distance;
      };
    }
    else // If manual or tag-seeking
    {
      target.offset = Translation2d.kZero;
      target.distance = distance;
    }

    target.altitude = switch (target.state)
    {
      case Manual -> target.altitude;
      case Point -> Interpolation.shooterAltitudeLow.get(target.distance);
      case Hub -> Interpolation.shooterAltitudeHub.get(target.distance);
      case Vision -> 0;
    };

    if (shootStatus == Status.Idling || shootStatus == Status.BadLocation)
      target.speed = FlywheelConstants.idleSpeed;
    else
      target.speed = switch (target.state)
      {
        case Manual -> target.speed;
        case Point -> Interpolation.flywheelSpeedLow.get(target.distance);
        case Hub -> Interpolation.flywheelSpeedHub.get(target.distance);
        case Vision -> FlywheelConstants.idleSpeed;
      };

    flywheels.update();
    turret.update(shooterPose, Math.toDegrees(swerveState.Speeds.omegaRadiansPerSecond));
    hood.update();

    updateStatus();
    telemetrise();
  }

  @Override
  public void simulationPeriodic() 
  {
    turret.updateSim();
    flywheels.updateSim();
  }
}
