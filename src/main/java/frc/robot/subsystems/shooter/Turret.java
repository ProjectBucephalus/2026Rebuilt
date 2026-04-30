package frc.robot.subsystems.shooter;

import frc.robot.constants.Constants.ShooterConstants.TurretConstants;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import static frc.robot.constants.Constants.ShooterConstants.TurretConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Interface class for a turret mechanism to control the azimuth of a shooter. <p>
 * Ensures rotation limits are respected to prevent damage to cables. <p>
 * Uses a TalonFXS controlled motor, and a potentiometer for calibration. <p>
 * Includes functionality to track a point on the field while the robot is in motion. <p>
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Turret
{
  private final TalonFXS m_Turret;
  private final AnalogPotentiometer io_Azimuth;
  @Logged
  private double deviceTemp;

  private final DCMotorSim motorSim = new DCMotorSim
  (
    LinearSystemId.createDCMotorSystem
      (DCMotor.getNeo550(1), 0.5, azimuthMotorRatio),
    DCMotor.getNeo550(1)
  );

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  private final Target target;

  private double lastCalibration = 0;
  private boolean azCheck = false;

  private CircularBuffer<Double> potBuffer = new CircularBuffer<>(5);

  /**
   * Creates a turret controller, to be managed by {@link Shooter} master-system
   * @param motorID CAN-ID of azimuth motor
   * @param potID AIO-ID of azimuth potentiometer
   * @param potOffset Potentiometer reading for centre of rotation
   * @param target Target object for the shooter
   */
  public Turret(int motorID, int potID, double potOffset, Target target) 
  {
    m_Turret = new TalonFXS(motorID);
    io_Azimuth = new AnalogPotentiometer(potID, potRange, potOffset);

    this.target = target;

    m_Turret.getConfigurator().apply(turretConfig);

    var simState = m_Turret.getSimState();
    simState.MotorOrientation = ChassisReference.Clockwise_Positive;
    simState.ExtSensorOrientation = ChassisReference.CounterClockwise_Positive;

    potBuffer.addFirst(io_Azimuth.get());

    calibrate();
  }

  /**
   * Get the current rotational speed of the Turret
   * 
   * @return the speed, in rotations per second
   */
  public double getSpeed()
  {
    if (RobotBase.isSimulation())
      return motorSim.getAngularVelocity().in(Units.RotationsPerSecond);
    else 
      return m_Turret.getVelocity().getValue().in(Units.RotationsPerSecond);
  }

  /** 
   * Get the current azimuth (rotational position) of the Turret
   * 
   * @return the azimuth, in degrees
   */
  @Logged(name = "azimuth Degrees")
  public double getAzimuth() 
  {
    if (RobotBase.isSimulation())
      return motorSim.getAngularPosition().in(Units.Degrees);
    else 
      return m_Turret.getPosition().getValue().in(Units.Degrees);
  }

  public Pair<Double, Double> getAzimuthTimestamped()
  {
    if (RobotBase.isSimulation())
    {
      double angle = motorSim.getAngularPosition().in(Units.Degrees);
      return new Pair<>(Utils.getCurrentTimeSeconds(), angle);
    }
    else 
    {
      var signal = m_Turret.getPosition();
      return new Pair<>(signal.getTimestamp().getTime(), signal.getValue().in(Units.Degrees));
    }
  }
  
  /** @return turret degrees as reported by potentiometer */
  @Logged(name = "potentiometer Degrees")
  public double getRawAzimuth()
    {return Conversions.round(io_Azimuth.get() / TurretConstants.azimuthPotRatio, 2);}

  /**
   * Unwind the turret by driving it one rotation towards zero
   */
  public void unwind()
  {
    double angle = getAzimuth();
    m_Turret.setControl(request.withPosition(Conversions.normaliseAngle(angle + 360, angle, maxTurretAzimuth) / 360)); 
  }

  /**
   * Drive the turret to it's robot-relative zero position
   */
  public void home()
    {m_Turret.setControl(request.withPosition(0));}

  /** @return {@code true} if potentiometer is in valid range */
  public boolean potValid() 
  {
    double rawAzimuth = io_Azimuth.get();
    return rawAzimuth >= -potSafeLimit && rawAzimuth <= potSafeLimit;
  }

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
    {return m_Turret.isConnected();}

  /**
   * If the turret is not moving, resets the motor's internal position to the current potentiometer reading
   */
  public void calibrate(boolean... force)
  {
    double rawAzimuth = io_Azimuth.get();
    potBuffer.addFirst(rawAzimuth);

    // If the turret is not moving fast and has moved since last calibration,
    // pull the value from the pot, convert to mechanism angle, and send to motor
    if 
    (
      RobotBase.isReal() 
      && Math.abs(getSpeed()) < calibrationSpeedLimit // Only calibrate when turret is moving slowly
      && (
        force.length != 0
        || !MathUtil.isNear(rawAzimuth, lastCalibration, calibrationAngleLimit) // Only calibrate after moving ~10 degrees
      )
      && potValid() // Discard extreme values that occur when sensor is disconnected
    )
    {
      double avg = 0;
      for (int i = 0; i < potBuffer.size(); i++)
        {avg += potBuffer.get(i);}
      avg /= potBuffer.size();

      double newPos = avg / (azimuthPotRatio * 360.0);
      m_Turret.setPosition(newPos);
      lastCalibration = avg;
    }
  }

  /**
   * Calculate the angle that the turret should rotate to in order to aim at the given target point
   * 
   * @param shooterPose the field-relative shooter pose
   * @param targetPoint the Translation2d of the target
   * @return the robot-relative target angle in degrees 
   */
  private static double calculateTargetAngle(Pose2d shooterPose, Translation2d targetPoint, double robotDegreesPerSecond )
  {
    // Angle from turret centre to target relative to field +X axis
    double fieldTarget = targetPoint.minus(shooterPose.getTranslation()).getAngle().getDegrees();
    // Robot-Relative angle from turret to target
    double robotTarget = fieldTarget - shooterPose.getRotation().getDegrees();

    double robotDegreesPerCycle = robotDegreesPerSecond / 50;

    return robotTarget - robotDegreesPerCycle;
  }

  private boolean safeToShoot(ChassisSpeeds swerveSpeeds)
  {
    return (getSpeed() - (Math.toDegrees(swerveSpeeds.omegaRadiansPerSecond)/360)) < maxRPS
      && Math.abs(getAzimuth()) < maxTurretAzimuth - limitBufferZone;
  }

  public boolean atAzimuth()
  {
    // Use given tolerance for reaching target, use double tolerance for no longer being at target
    if (Conversions.nearRotation(getAzimuth(), target.azimuth, azimuthTolerance))
      azCheck = true;
    else if (!Conversions.nearRotation(getAzimuth(), target.azimuth, azimuthTolerance * 2))
      azCheck = false;

    return azCheck;
  }

  /** 
   * Checks that the turret system is in a safe and valid state to begin shooting. This requires that:
   * <ul>
   * <li> The turret is within {@link TurretConstants#azimuthTolerance azimuthTolerance} of it's target azimuth
   * <li> The combined rotational velocity of the turret and the drivebase is less than {@link TurretConstants#maxRPS maxRPS}  
   * <li> The turret is not within {@link TurretConstants#limitBufferZone limitBufferZone} of it's max azimuth
   * </ul>
   * 
   * @return True if all above conditions are true
   */
  public boolean readyToShoot(ChassisSpeeds swerveSpeeds)
    {return atAzimuth() && safeToShoot(swerveSpeeds);}

  /**
   * Intended to be called in {@link Shooter#periodic()} <p>
   * Recalculate the target azimuth and apply it to the motor
   * 
   * @param shooterPose the field-relative shooter pose
   * @param robotDegreesPerSecond the current rate of rotation of the drivebase
   */
  protected void update(Pose2d shooterPose, double robotDegreesPerSecond)
  {
    calibrate();

    if (target.disabled)
    {
      target.azimuth = getAzimuth();
      m_Turret.set(0);
    }
    else
    {
      if (target.state != TargetState.Manual)
      {
        // Seek tags when traversing bump or trench
        if (GeoFencing.obstacleBlue.checkPosition(shooterPose.getTranslation()))
        { // If turret mount is facing towards Neutral zone from Blue obstacle, look for Red Hub tags
          if (Conversions.nearRotation(shooterPose.getRotation(), Rotation2d.kZero, 90))
            {target.azimuth = calculateTargetAngle(shooterPose, GeoFencing.hubRed.getCentre(), robotDegreesPerSecond);}
          else // If turret mount is facing towards Blue zone from Blue obstacle, look for Blue Tower tags
            {target.azimuth = calculateTargetAngle(shooterPose, GeoFencing.towerBlue.getCentre(), robotDegreesPerSecond);}
        }
        else if (GeoFencing.obstacleRed.checkPosition(shooterPose.getTranslation()))
        {
          // If turret mount is facing towards Neutral zone from Red obstacle, look for Blue Hub tags
          if (Conversions.nearRotation(shooterPose.getRotation(), Rotation2d.k180deg, 90))
            {target.azimuth = calculateTargetAngle(shooterPose, GeoFencing.hubBlue.getCentre(), robotDegreesPerSecond);}
          else // If turret mount is facing towards Red zone from Red obstacle, look for Red Tower tags
            {target.azimuth = calculateTargetAngle(shooterPose, GeoFencing.towerRed.getCentre(), robotDegreesPerSecond);}
        }
        // Update the azimuth stored in the target based on the target state
        // Ensures that changing to manual mode doesn't cause sudden motion
        else if (target.state == TargetState.Hub)
          target.azimuth = calculateTargetAngle(shooterPose, FieldUtils.getAllianceHubCentre().plus(target.offset), robotDegreesPerSecond);
        else if (target.state == TargetState.Point)
          target.azimuth = calculateTargetAngle(shooterPose, target.point.plus(target.offset), robotDegreesPerSecond);
      }

      m_Turret.setControl(request.withPosition(Conversions.normaliseAngle(target.azimuth, getAzimuth(), maxTurretAzimuth) / 360));
    }

    deviceTemp = Math.max(m_Turret.getDeviceTemp().getValueAsDouble(), m_Turret.getProcessorTemp().getValueAsDouble());
  }   
  
  protected void updateSim()
  {
    var motorSimState = m_Turret.getSimState();
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = motorSimState.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSim.setInputVoltage(motorVoltage.in(Units.Volts));
    motorSim.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    motorSimState.setRawRotorPosition(motorSim.getAngularPosition().times(azimuthMotorRatio));
    motorSimState.setRotorVelocity(motorSim.getAngularVelocity().times(azimuthMotorRatio));
  }
}