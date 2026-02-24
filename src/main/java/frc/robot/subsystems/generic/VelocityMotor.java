package frc.robot.subsystems.generic;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * A subsystem wrapped around a TalonFX to provide a simple subsystem for controlling a motor with mechanism ratio
 * @author 5985
 */
public class VelocityMotor extends SubsystemBase 
{
  private final TalonFX m_Velocity;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

  /**
   * Creates a wrapper around a TalonFX to provides simple binary control (on or off)
   * 
   * @param defaultSpeed the duty-cycle speed to run at when on [-1..1]
   * @param id the id of the motor
   */
  public VelocityMotor(int id, TalonFXConfiguration config) 
  {
    m_Velocity = new TalonFX(id);
    applyConfig(config);
  }

  /**
   * Applies a new configuration object to the motor
   * 
   * @param config the configuration to apply
   * @return this subsystem, for easier chaining
   */
  public VelocityMotor applyConfig(TalonFXConfiguration config) 
  {
    m_Velocity.getConfigurator().apply(config);
    return this;
  }

  /**
   * Set the speed of the motor
   * 
   * @param speed the desired speed, in mechanism rotations per second
   */
  public void setSpeed(double speed)
    {m_Velocity.setControl(request.withVelocity(speed));}

  /**
   * Construct a command that sets the speed of the motor <p>
   * NOTE: The provided value is only evaluated when the command is created
   * 
   * @param speed the desired speed, in mechanism rotations per second
   * @return the {@link Command}
   */
  public Command setSpeedCommand(double speed)
    {return runOnce(() -> setSpeed(speed));}

  /** @return Current speed of the motor, in mechanism rotations per second */
  public double getSpeed() 
    {return m_Velocity.getVelocity().getValueAsDouble();}

  public double getTemp() 
    {return m_Velocity.getAncillaryDeviceTemp().getValueAsDouble();}

  public double getMotorCurrent()
    {return m_Velocity.getStatorCurrent().getValueAsDouble();}
}
