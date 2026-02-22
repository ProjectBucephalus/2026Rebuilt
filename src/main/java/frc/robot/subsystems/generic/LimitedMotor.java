package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
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

  private final double stallCurrent;

  private final boolean slot1Valid;
  private final boolean useStall;

  private boolean homed = false;
  private boolean homeLastCycle = false;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  /**
   * Creates generic limited motor system
   * @param motorCAN CAN-ID of underlying motor
   * @param limitIO DIO-ID of home limit-sensor. Set to {@code -1} to use motor stall instead of limit switch
   * @param minRotations Minimum position in mechanism rotations
   * @param maxRotations Maximum position in mechanism rotations
   * @param configs Motor configuration object, uses Slot1 if present when not calibrated <br>
   *                {@code CustomParam0} is used for the stall current value if using motor stall
   */
  public LimitedMotor(int motorCAN, int limitIO, double minRotations, double maxRotations, TalonFXConfiguration configs)
  {
    this.maxRotations = maxRotations;
    this.minRotations = minRotations;
    slot1Valid = configs.Slot1.kP != 0;
    useStall = limitIO == -1;
    stallCurrent = configs.CustomParams.CustomParam0;

    m_Inner = new TalonFX(motorCAN);
    io_Limit = new DigitalInput(limitIO);

    m_Inner.getConfigurator().apply(configs);

    m_Inner.setPosition(maxRotations);
  } 

  public boolean atLimit() 
  {
    return useStall 
      ? m_Inner.getStatorCurrent().getValueAsDouble() >= stallCurrent
      : io_Limit.get();
  }

  /** @return Current physical angle, in mechanism rotations */
  public double getAngle()
    {return m_Inner.getPosition().getValue().in(Units.Rotation);}

  /**
   * Sets the target point for the motor 
   * @param target mechanism rotations
   */
  public void setTarget(double target) 
  {
    double clampedRotations = Conversions.clamp(target, minRotations, maxRotations);
    int slot = !homed && slot1Valid ? 1 : 0;
    m_Inner.setControl
      (request.withPosition(clampedRotations).withSlot(slot));
  }

  /**
   * Creates a command to set the target point for the motor <p>
   * NOTE: The provided value is only evaluated when the command is created
   * @param target mechanism rotations
   * @return the Command
   */
  public Command setTargetCommand(double target)
    {return runOnce(() -> setTarget(target));}

  /**
   * Creates a command to continuously adjust the target point of the motor by a dynamic amount <p>
   * Primarily intended for joystick control
   * @param shiftSup A supplier for the amount to adjust the target by in mechanism rotations
   * @return the Command
   */
  public Command adjustTargetCommand(DoubleSupplier shiftSup) 
    {return run(() -> setTarget(getAngle() + shiftSup.getAsDouble()));}

  @Override
  public void periodic() 
  {
    if (atLimit())
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
