package frc.robot.subsystems.shooter;

import frc.robot.constants.Constants.ShooterConstants.TurretConstants;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import static frc.robot.constants.Constants.ShooterConstants.TurretConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * Interface class for a turret mechanism to control the azimuth of a shooter. <p>
 * Ensures rotation limits are respected to prevent damage to cables. <p>
 * Uses a TalonFX controlled motor, and a potentiometer for calibration. <p>
 * Includes functionality to track a point on the field while the robot is in motion. <p>
 * @author 5985
 */
public class Turret
{
  private final TalonFX m_Turret;
  private final AnalogPotentiometer io_Azimuth;

  private final Supplier<Target> targetSup;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  /**
   * Creates a turret controller, to be managed by {@link Shooter} master-system
   * @param motorID CAN-ID of azimuth motor
   * @param potID AIO-ID of azimuth potentiometer
   * @param potOffset Potentiometer reading for centre of rotation
   * @param targetSup Supplier for current Target object
   */
  public Turret(int motorID, int potID, double potOffset, Supplier<Target> targetSup) 
  {
    m_Turret = new TalonFX(motorID);
    io_Azimuth = new AnalogPotentiometer(potID, TurretConstants.potRange, potOffset);

    this.targetSup = targetSup;

    m_Turret.getConfigurator().apply(turretConfig);

    calibrate();
  }

  /**
   * Get the current rotational speed of the Turret
   * 
   * @return the speed, in rotations per second
   */
  public double getSpeed()
    {return m_Turret.getVelocity().getValueAsDouble();}

  /** 
   * Get the current azimuth (rotational position) of the Turret
   * 
   * @return the azimuth, in degrees
   */
  public double getAzimuth() 
    {return m_Turret.getPosition().getValue().in(Units.Degrees);}

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

  /**
   * If the turret is not moving, resets the motor's internal position to the current potentiometer reading
   */
  public void calibrate()
  {
    // If the turret is not moving, pull the value from the pot, convert to mechanism angle, and send to motor
    if (Math.abs(m_Turret.getVelocity().getValueAsDouble()) < 0.1) // TODO Put this in constants
      m_Turret.setPosition((io_Azimuth.get() * azimuthGearRatio) / 360.0);
  }

  /**
   * Calculate the angle that the turret should rotate to in order to aim at the given target point
   * 
   * @param shooterPose the field-relative shooter pose
   * @param targetPoint the Translation2d of the target
   * @return the robot-relative target angle in degrees 
   */
  private double calculateTargetAngle(Pose2d shooterPose, Translation2d targetPoint, double robotDegreesPerSecond )
  {
    // Angle from turret centre to target relative to field +X axis
    double fieldTarget = targetPoint.minus(shooterPose.getTranslation()).getAngle().getDegrees();
    // Robot-Relative angle from turret to target
    double robotTarget = fieldTarget - shooterPose.getRotation().getDegrees();

    double robotDegreesPerCycle = robotDegreesPerSecond / 50;

    return Conversions.normaliseAngle(robotTarget - robotDegreesPerCycle, getAzimuth(), maxTurretAzimuth);
  }

  public boolean safeToShoot(ChassisSpeeds swerveSpeeds)
  {
    return (getSpeed() + (Math.toDegrees(swerveSpeeds.omegaRadiansPerSecond)/360)) < TurretConstants.maxRPS;
  }

  public boolean atAzimuth()
  {
    return Conversions.nearRotation(getAzimuth(), targetSup.get().azimuth, TurretConstants.azimuthTolerance);
  }

  /**
   * Intended to be called in {@link Shooter#periodic()} <p>
   * Recalculate the target azimuth and apply it to the motor
   * 
   * @param shooterPose the field-relative shooter pose
   * @param robotDegreesPerSecond the current rate of rotation of the drivebase
   */
  public void update(Pose2d shooterPose, double robotDegreesPerSecond)
  {
    calibrate();
    var target = targetSup.get();
    // Update the azimuth stored in the target based on the target state
    // Ensures that changing to manual mode doesn't cause sudden motion
    target.azimuth = switch (target.state) 
    {
      case Manual -> target.azimuth;
      case Point -> calculateTargetAngle(shooterPose, target.point.plus(target.offset), robotDegreesPerSecond);
      // aim at our alliance's hub
      case Hub -> calculateTargetAngle(shooterPose, FieldUtils.getAllianceHubCentre().plus(target.offset), robotDegreesPerSecond);
    };

    m_Turret.setControl(request.withPosition(target.azimuth / 360));
  }   
}