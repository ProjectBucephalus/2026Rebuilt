package frc.robot.subsystems.generic;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * A subsystem wrapped around a TalonFX to provide a simple subsystem for controlling a motor with mechanism ratio
 * @author 5985
 */
public class VelocityMotor extends SubsystemBase 
{
  protected final TalonFX m_Velocity;

  protected final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

  /**
   * Creates a wrapper around a TalonFX to provide velocity control
   * 
   * @param id the id of the motor
   * @param config the config to apply to the wrapped motor
   */
  public VelocityMotor(int id, TalonFXConfiguration config) 
  {
    m_Velocity = new TalonFX(id);
    m_Velocity.getConfigurator().apply(config);
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
  @Logged(name = "Speed Rotations per Second")
  public double getSpeed() 
    {return m_Velocity.getVelocity().getValue().in(Units.RotationsPerSecond);}

  @Logged(name = "Temp Celsius")
  public double getTemp() 
    {return m_Velocity.getAncillaryDeviceTemp().getValue().in(Units.Celsius);}

  @Logged(name = "Current Amps")
  public double getMotorCurrent()
    {return m_Velocity.getStatorCurrent().getValue().in(Units.Amps);}
}
