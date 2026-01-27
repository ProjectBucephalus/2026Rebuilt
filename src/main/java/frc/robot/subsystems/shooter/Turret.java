package frc.robot.subsystems.shooter;

import frc.robot.constants.Constants;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import static frc.robot.constants.Constants.TurretConstants.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Turret
{
  private final TalonFX m_Turret;
  private final AnalogPotentiometer io_Azimuth;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  private final Translation2d robotOffset;

  public Turret(int motorID, int potID, double potOffset, Translation2d robotOffset) 
  {
    this.robotOffset = robotOffset;
    m_Turret = new TalonFX(motorID);
    io_Azimuth = new AnalogPotentiometer(potID, Constants.TurretConstants.maxTurretAzimuth, potOffset);

    m_Turret.getConfigurator().apply(turretConfigs);

    calibrate();
  }

  /**
   * @param robotPose Current Pose2d of the robot
   * @param targetPoint Translation2d of the target
   * @return Desired turret angle to aim at target, in degrees
   */
  private double calculateTargetAngle(Pose2d robotPose, Translation2d targetPoint)
  {
    var turretPos = robotPose.getTranslation().plus(robotOffset);
    // Angle from turret centre to target relative to field +X axis
    double fieldTarget = targetPoint.minus(turretPos).getAngle().getDegrees();
    // Robot-Relative angle from turret to target
    double robotTarget = fieldTarget - robotPose.getRotation().getDegrees();

    return Conversions.normaliseAngle(robotTarget, getRotation(), maxTurretAzimuth);
  }

  public void unwind()
  {
    double angle = getRotation();
    m_Turret.setControl(request.withPosition(Conversions.normaliseAngle(angle + 360, angle, maxTurretAzimuth) / 360)); 
  }

  public void home()
    {m_Turret.setControl(request.withPosition(0));}

  /** Returns mechanism rotation in degrees */
  public double getRotation() 
    {return m_Turret.getPosition().getValue().in(Units.Degrees);} // TODO Define and standadise rotation direction
  
  //using the states defined in Target.java to set the place that the turret is tracking
  public void update(Pose2d robotPose, Target target)
  {
    calibrate();
    
    double targetAzimuth = switch (target.state) 
    {
      case Manual -> target.azimuth;
      case Point -> calculateTargetAngle(robotPose, target.point);
      case Hub -> calculateTargetAngle(robotPose, FieldUtils.getAllianceHubCentre());
    };

    target.azimuth = targetAzimuth;

    // gives control of the motors to request to be called in Robot.java
    m_Turret.setControl(request.withPosition(targetAzimuth / 360));     
  }   

  public void calibrate()
  {
    // If the turret is not moving, pull the value from the pot, convert to mechanism angle, and send to motor
    if (Math.abs(m_Turret.getVelocity().getValueAsDouble()) < 0.1)
    {
      m_Turret.setPosition((io_Azimuth.get()*gearRatio)/360.0);
    }
  }
}