package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Conversions;
/** 
 * Generic subclass for a range-limited motor with a binary switch at the home position 
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class LimitedMotor extends PositionMotor
{
  private final Limit limit;

  private final double maxRotations;
  private final double minRotations;
  private final double homeRotations;

  private final boolean slot1Valid;

  private boolean homed = false;
  private boolean homeLastCycle = false;

  /**
   * Creates generic limited motor system
   * @param motorCAN CAN-ID of underlying motor
   * @param limitIO DIO-ID of home limit-sensor. Set to {@code -1} to use motor stall instead of limit switch
   * @param minRotations Minimum position in mechanism rotations
   * @param maxRotations Maximum position in mechanism rotations
   * @param homeRotations Sensor trigger position in mechanism rotations
   * @param configs Motor configuration object, uses Slot1 if present when not calibrated <br>
   *                {@code CustomParam0} is used for the stall current value if using motor stall
   */
  public LimitedMotor(int motorCAN, int limitIO, double minRotations, double maxRotations, double homeRotations, TalonFXConfiguration configs)
  {
    super(motorCAN, configs);
    this.maxRotations = maxRotations;
    this.minRotations = minRotations;
    this.homeRotations = homeRotations;
    slot1Valid = configs.Slot1.kP != 0;

    m_Position.setPosition(minRotations);

    if (limitIO == -1)
      limit = new StallLimit(configs.CustomParams.CustomParam0);
    else
      limit = new DIOLimit(limitIO);
  } 

  /**
   * Sets the target point for the motor 
   * @param target mechanism rotations
   */
  @Override
  public void setTarget(double target) 
  {
    double clampedRotations = Conversions.clamp(target, minRotations, maxRotations);
    int slot = (!homed && slot1Valid) ? 1 : 0;
    request.withSlot(slot);
    super.setTarget(clampedRotations);
  }

  /**
   * Sets the target point for the motor, ignoring limits
   * @param target mechanism rotations
   */
  public void forceSetTarget(double target)
  {
    request.withSlot(0);
    super.setTarget(target);
  }

  /** Sets the target to the maximum limit */
  public Command deployCommand() 
    {return setTargetCommand(maxRotations);}
  /** Sets the target to the minimum limit */
  public Command retractCommand() 
    {return setTargetCommand(minRotations);}

  /**
   * Creates a command to continuously adjust the target point of the motor by a dynamic amount <p>
   * Primarily intended for joystick control
   * @param shiftSup A supplier for the amount to adjust the target by in mechanism rotations
   * @return the Command
   */
  @Override
  public Command adjustTargetCommand(DoubleSupplier shiftSup) 
    {return run(() -> {if (shiftSup.getAsDouble() != 0) forceSetTarget(getAngle() + shiftSup.getAsDouble());});}

  @Override
  public void periodic() 
  {
    if (limit.atLimit())
    {  
      if (!homeLastCycle && !homed)
      {
        homed = true;
        homeLastCycle = true;
        m_Position.setPosition(homeRotations);
      }
    }
    else 
      homeLastCycle = false;
  }

  private interface Limit 
    {boolean atLimit();}

  private class DIOLimit implements Limit 
  {
    private final DigitalInput io_Limit;

    public DIOLimit(int limitIO)
      {io_Limit = new DigitalInput(limitIO);}

    @Override
    public boolean atLimit() 
      {return io_Limit.get();}
  }

  private class StallLimit implements Limit 
  {
    private final double stallCurrent;

    public StallLimit(double stallCurrent)
      {this.stallCurrent = stallCurrent;}

    @Override
    public boolean atLimit() 
      {return Math.abs(m_Position.getTorqueCurrent().getValueAsDouble()) >= stallCurrent;}
  }
}
