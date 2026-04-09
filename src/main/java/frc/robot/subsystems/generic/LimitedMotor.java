package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Conversions;
/** 
 * Generic subclass for a range-limited motor with a binary switch reading {@code true} at the home position 
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class LimitedMotor extends PositionMotor
{
  @Logged
  private final Limit limit;

  private final double minRotations;
  private final double maxRotations;
  private final double homeRotations;

  private final boolean slot1Valid;
  private boolean sensorValid = false;
  private boolean active = false;

  private boolean calibrated = false;
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
    this.maxRotations = Math.max(maxRotations, minRotations);
    this.minRotations = Math.min(maxRotations, minRotations);
    this.homeRotations = Conversions.clamp(homeRotations, maxRotations, minRotations);
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
    // If valid, use the second PID slot until the mechanism has been homed
    int slot = (!calibrated && slot1Valid) ? 1 : 0;
    request.withSlot(slot);
    active = true;
    super.setTarget(clampedRotations);
  }

  /** @return Command to set the target to the maximum limit */
  public Command extendCmd() 
    {return setTargetCmd(maxRotations);}
  /** @return Command to set the target to the minimum limit */
  public Command retractCmd() 
    {return setTargetCmd(minRotations);}

  // /**
  //  * Creates a command to continuously adjust the target point of the motor by a dynamic amount <p>
  //  * Primarily intended for joystick control, ignoring limits
  //  * @param shiftSup A supplier for the amount to adjust the target by in mechanism rotations
  //  * @return the Command
  //  */
  // @Override
  // public Command adjustTargetCmd(DoubleSupplier shiftSup) 
  //   {return run(() -> {if (shiftSup.getAsDouble() != 0) super.setTarget(getAngle() + shiftSup.getAsDouble());});}

  @Override
  public void periodic() 
  {
    if (!sensorValid && active)
    {
      // First cycle active becomes true, marking that the sensor is now definitely valid
      sensorValid = true;

      var range = maxRotations - minRotations;
      double tolerance = range / 20;

      if (limit.atLimit())
      {
        // If the switch is initially true:
        //  If home poisition is close to an end, set the position to that endpoint
        //  Otherwise set the position slightly below home
        if (homeRotations <= minRotations + tolerance) 
          m_Position.setPosition(minRotations);
        else if (homeRotations >= maxRotations - tolerance) 
          m_Position.setPosition(maxRotations);
        else 
          m_Position.setPosition(homeRotations - tolerance);
      }
      else
      {
        // If the switch is initially false:
        //  If home poisition is close to an end, set the position to the other endpoint
        //  Otherwise set the position to the midpoint of the range of motion
        if (homeRotations <= minRotations + tolerance) 
          m_Position.setPosition(maxRotations);
        else if (homeRotations >= maxRotations - tolerance) 
          m_Position.setPosition(minRotations);
        else
          m_Position.setPosition((minRotations + maxRotations) / 2);
      }
    }

    if (active)
    {
      if (limit.atLimit())
      {  
        // Flag when the sensor is true
        homeLastCycle = true;
      }
      else
      {
        if (homeLastCycle && !calibrated)
        {
          // When the sensor first *becomes* false, calibrate
          calibrated = true;
          m_Position.setPosition(homeRotations);
        }

        homeLastCycle = false;
      }
    }
  }

  @Logged
  public interface Limit 
    {public boolean atLimit();}

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

