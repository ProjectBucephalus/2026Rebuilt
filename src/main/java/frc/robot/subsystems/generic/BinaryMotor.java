package frc.robot.subsystems.generic;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * A subsystem wrapped around a TalonFX to provide a simple subsystem for any motor mainly intended for binary operation (on or off) 
 * @author 5985
 */
public class BinaryMotor extends SubsystemBase 
{
  private final TalonFX m_Binary;
  private final double defaultSpeed;

  /**
   * Creates a wrapper around a TalonFX to provides simple binary control (on or off)
   * 
   * @param defaultSpeed the duty-cycle speed to run at when on [-1..1]
   * @param id the id of the motor
   */
  public BinaryMotor(int id, double defaultSpeed) 
  {
    this.defaultSpeed = defaultSpeed;
    m_Binary = new TalonFX(id);
  }

  public double getSpeed()
  {
    return m_Binary.getVelocity().getValueAsDouble();
  }

  public BinaryMotor(int id, double defaultSpeed, TalonFXConfiguration config) 
  {
    this.defaultSpeed = defaultSpeed;
    m_Binary = new TalonFX(id);
    applyConfig(config);
  }

  public BinaryMotor applyConfig(TalonFXConfiguration config) 
  {
    m_Binary.getConfigurator().apply(config);
    return this;
  }

  /**
   * Sets the speed of the motor to an arbitrary value <p>
   * 
   * @param speed the duty-cycle speed to run at [-1..1]
   */
  public void setSpeed(double speed) 
    {m_Binary.set(speed);}

  public void start()
    {m_Binary.set(defaultSpeed);}

  public void stop()
    {m_Binary.set(0);}

  /**
   * Construct a command that runs the motor at the default speed on start, and stops the motor on end
   * 
   * @return the {@link Command}
   */
  public Command runCmd()
    {return startEnd(this::start, this::stop);}

  /**
   * Construct a command that runs the motor at the default speed
   * 
   * @return the {@link Command}
   */
  public Command startCmd()
    {return runOnce(this::start);}
  
  /**
   * Construct a command that runs the motor at negative default speed
   * 
   * @return the {@link Command}
   */
  public Command reverseCmd()
    {return runOnce(() -> m_Binary.set(-defaultSpeed));}

  /**
   * Construct a command that stops the motor (i.e., sets speed to 0) 
   * 
   * @return the {@link Command}
   */
  public Command stopCmd()
    {return runOnce(this::stop);}

  /**
   * Construct a command that sets the speed of the motor to an arbitrary value <p>
   * NOTE: The provided value is only evaluated when the command is created
   * 
   * @param speed the duty-cycle speed to run at [-1..1]
   * @return the {@link Command}
   */
  public Command setSpeedCmd(double speed)
    {return runOnce(() -> m_Binary.set(speed));}

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
    {return m_Binary.isConnected();}
}
