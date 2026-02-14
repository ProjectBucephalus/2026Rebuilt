package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;

import static frc.robot.constants.Constants.ShooterConstants.FlywheelConstants.*;
import frc.robot.constants.Constants.ShooterConstants;

/**
 * Interface class for a shooter flywheel. <p>
 * Uses two linked TalonFX controlled motors. <p>
 * Uses MotionMagic to control velocity.
 * @author 5985
 */
public class Flywheels 
{
  private final TalonFX m_Leader; 
  private final TalonFX m_Follower;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

  /**
   * Creates a velocity controlled flywheel, to be managed by {@link ShooterConstants} master-system
   * @param leaderCAN CAN-ID of primary shooter motor
   * @param followerCAN CAN-ID of secondary shooter motor, set to follow first
   */
  public Flywheels(int leaderCAN, int followerCAN)
  {
    m_Leader = new TalonFX(leaderCAN);
    m_Follower = new TalonFX(followerCAN);

    m_Leader.getConfigurator().apply(flywheelConfig);
    m_Follower.getConfigurator().apply(flywheelConfig);

    m_Follower.setControl(new Follower(leaderCAN, MotorAlignmentValue.Opposed));
  }

  /**
   * Set the motor speed
   * 
   * @param speed the desired speed, in mechanism rotations per second
   */
  public void setSpeed(double speed)
    {m_Leader.setControl(request.withVelocity(speed));}

  /**
   * Checks if the current motor speed is within {@link ShooterConstants#flySpeedTolerance flySpeedTolerance} of the requested speed
   * 
   * @return true if the motor is at speed
   */
  public boolean atSpeed() 
  {
    return MathUtil.isNear(request.Velocity, getSpeed(), flySpeedTolerance);
  }

  public double getSpeed() 
    {return m_Leader.getVelocity().getValueAsDouble();}

  public double getTemp() 
    {return m_Leader.getAncillaryDeviceTemp().getValueAsDouble();}

  public double getMotorCurrent()
    {return m_Leader.getStatorCurrent().getValueAsDouble();}

  public void update()
  {

  }

  public void setVoltage(double voltage)
  {
    m_Leader.setVoltage(voltage);
  }
}
