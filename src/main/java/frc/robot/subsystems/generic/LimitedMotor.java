package frc.robot.subsystems.generic;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  protected final double minRotations;
  protected final double maxRotations;
  protected final double homeRotations;

  private final boolean slot1Valid;

  private boolean sensorValid = false;
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
    super.setTarget(clampedRotations);
  }

  /** @return Command to set the target to the maximum limit */
  public Command extendCmd() 
    {return setTargetCmd(maxRotations);}
  /** @return Command to set the target to the minimum limit */
  public Command retractCmd() 
    {return setTargetCmd(minRotations);}

  /** @return Command to run basic calibration cycle, calibrating on the high edge of the sensor if possible */
  public Command calibrateCmd()
  {
    SequentialCommandGroup calCmd = new SequentialCommandGroup
    (
      new SequentialCommandGroup
      (
        setTargetCmd(homeRotations),
        waitUntil(this::atTarget),
        setTargetCmd((maxRotations + minRotations) / 2),
        waitUntil(this::atTarget),
        setTargetCmd(minRotations),
        waitUntil(this::atTarget),
        setTargetCmd(maxRotations),
        waitUntil(this::atTarget)
      )
        .until(limit::atLimit),
      
      adjustTargetCmd(() -> homeRotations == maxRotations ? -0.05 : 0.05)
        .until(() -> calibrated),
      setTargetCmd(homeRotations)
    );

    return calCmd;
  }

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
        if (!homeLastCycle && !calibrated)
        {
          // When the sensor *becomes* true while not calibrated, set position without flagging as calibrated
          m_Position.setPosition(homeRotations);
        }
        // Flag when the sensor is true
        homeLastCycle = true;

        // If sensor is at endstop, stop
        if 
        (
          (homeRotations == minRotations && request.getPositionMeasure().in(Rotations) <= getAngle())
          ||
          (homeRotations == maxRotations && request.getPositionMeasure().in(Rotations) >= getAngle())
        )
        {
          stop();
        }
      }
      else // if not at limit
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

  /** @deprecated If mechanical makes you use this, insist that they add a sensor */
  @Deprecated
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

