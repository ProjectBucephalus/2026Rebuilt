package frc.robot.subsystems.generic;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Conversions;

/** 
 * Generic subclass for a range-limited motor with a binary switch at the home position 
 * @author 5985
 */
public class LimitedMotor extends SubsystemBase
{
  private final TalonFX m_Inner;
  private final DigitalInput io_Limit;

  private final double maxRotations;
  private final double minRotations;

  private final boolean slot1Valid;

  private boolean homed = false;
  private boolean homeLastCycle = false;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  /**
   * Creates generic limited motor system
   * @param motorCAN CAN-ID of underlying motor
   * @param limitIO DIO-ID of home limit-sensor
   * @param minRotations Minimum position in mechanism rotations
   * @param maxRotations Maximum position in mechanism rotations
   * @param configs Motor configuration object, uses Slot1 if present when not calibrated
   */
  public LimitedMotor(int motorCAN, int limitIO, double minRotations, double maxRotations, TalonFXConfiguration configs)
  {
    this.maxRotations = maxRotations;
    this.minRotations = minRotations;
    slot1Valid = configs.Slot1.kP != 0;

    m_Inner = new TalonFX(motorCAN);
    io_Limit = new DigitalInput(limitIO);

    m_Inner.getConfigurator().apply(configs);

    m_Inner.setPosition(maxRotations);
  } 

  /**
   * Creates a command to set the target point for the motor <p>
   * NOTE: The provided value is only evaluated when the command is created
   * @param targetRotations mechanism rotations
   * @return the Command
   */
  public Command setTargetCommand(double targetRotations)
  {
    return runOnce(() -> {
      double clampedRotations = Conversions.clamp(targetRotations, minRotations, maxRotations);
      int slot = !homed && slot1Valid ? 1 : 0;
      m_Inner.setControl
        (request.withPosition(clampedRotations).withSlot(slot));
    });
  }
  
  @Override
  public void periodic() 
  {
    if (io_Limit.get())
    {  
      if (!homeLastCycle)
      {
        homed = true;
        homeLastCycle = true;
        m_Inner.setPosition(0);
      }
    }
    else 
      homeLastCycle = false;
  }
}
