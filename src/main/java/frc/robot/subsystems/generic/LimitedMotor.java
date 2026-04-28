package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private final boolean safeSpeedValid;
  /** Tolerance to be considered at a position, calculated as 5% of travel range */
  private final double tolerance;

  private boolean sensorValid = false;
  private boolean calibrated = false;
  private boolean homeLastCycle = false;

  private TalonFXConfiguration motorConfig;

  /**
   * Creates generic limited motor system
   * @param motorCAN CAN-ID of underlying motor
   * @param limitIO DIO-ID of home limit-sensor. Set to {@code -1} to use motor stall instead of limit switch
   * @param invertLimit Whether the limit switch is inverted (i.e. false is at limit). Ignored if using motor stall
   * @param minRotations Minimum position in mechanism rotations
   * @param maxRotations Maximum position in mechanism rotations
   * @param homeRotations Sensor trigger position in mechanism rotations
   * @param configs Motor configuration object, uses Slot1 if present when not calibrated <br>
   *                {@code CustomParam0} is used for the stall current value if using motor stall
   */
  public LimitedMotor(int motorCAN, int limitIO, boolean invertLimit, double minRotations, double maxRotations, double homeRotations, TalonFXConfiguration configs)
  {
    super(motorCAN, configs);

    motorConfig = configs;

    // Sanitise inputs
    this.maxRotations = Math.max(maxRotations, minRotations);
    this.minRotations = Math.min(maxRotations, minRotations);
    this.homeRotations = Conversions.clamp(homeRotations, maxRotations, minRotations);

    tolerance = (this.maxRotations - this.minRotations) / 20;

    // If a valid CustomParam0 is provided, we want to use it for moving slower when not clibrated
    safeSpeedValid = configs.CustomParams.CustomParam0 != 0;

    if (safeSpeedValid)
    {
      var uncalibratedConfig = motorConfig.clone();
      uncalibratedConfig.MotionMagic.MotionMagicCruiseVelocity = uncalibratedConfig.CustomParams.CustomParam0 / 100.0;
      m_Position.getConfigurator().apply(uncalibratedConfig);
    }

    // Initialise motor position to a safe guess, will be further refined once we start moving and determined accurately once calibrated
    m_Position.setPosition(minRotations);

    // Allows using motor stall as limit. Deprecated
    if (RobotBase.isSimulation())
      limit = new SimLimit();
    else if (limitIO == -1)
      limit = new StallLimit(configs.CustomParams.CustomParam1);
    else
      limit = new DIOLimit(limitIO, invertLimit);
  } 

  public boolean atLimit()
    {return limit.atLimit();}

  public boolean atMax()
    {return MathUtil.isNear(maxRotations, getAngle(), tolerance);}

  public boolean atMin()
    {return MathUtil.isNear(minRotations, getAngle(), tolerance);}

  /**
   * Sets the target point for the motor 
   * @param target mechanism rotations
   */
  @Override
  public void setTarget(double target) 
  {
    double clampedRotations = Conversions.clamp(target, minRotations, maxRotations);
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
    return Commands.sequence
    (
      Commands.sequence
      (
        gotoTargetCmd(homeRotations),
        gotoTargetCmd((maxRotations + minRotations) / 2),
        gotoTargetCmd(minRotations),
        gotoTargetCmd(maxRotations)
      ).until(this::atLimit),
      adjustTargetCmd(() -> homeRotations == maxRotations ? -0.05 : 0.05).until(() -> calibrated),
      setTargetCmd(homeRotations)
    );
  }

  @Override
  public Command adjustTargetCmd(DoubleSupplier shiftSup) 
  {return run(() -> 
    {
      if 
      (
        shiftSup.getAsDouble() != 0
        && 
        !(
          atLimit()
          &&
          (
            (homeRotations <= minRotations + tolerance && shiftSup.getAsDouble() <= 0)
            ||
            (homeRotations >= maxRotations - tolerance && shiftSup.getAsDouble() >= 0)
          )
        )
        && 
        !(
          calibrated 
          && 
          (
            (shiftSup.getAsDouble() >= 0 && getAngle() >= maxRotations)
            ||
            (shiftSup.getAsDouble() <= 0 && getAngle() <= minRotations)
          )
        )
      ) 
        baseSetTarget(getAngle() + shiftSup.getAsDouble());
    }
  );}

  @Override
  public void periodic() 
  {
    // Initialise position when we first move
    if (!sensorValid && active)
    {
      // First cycle active becomes true, marking that the sensor is now definitely valid
      sensorValid = true;

      double position;
      if (atLimit())
      {
        // If the switch is initially true:
        //  If home poisition is close to an end, set the position to that endpoint
        //  Otherwise set the position slightly below home
        if (homeRotations <= minRotations + tolerance) 
          position = minRotations;
        else if (homeRotations >= maxRotations - tolerance) 
          position = maxRotations;
        else 
          position = homeRotations - tolerance;
      }
      else
      {
        // If the switch is initially false:
        //  If home poisition is close to an end, set the position to the other endpoint
        //  Otherwise set the position to the midpoint of the range of motion
        if (homeRotations <= minRotations + tolerance) 
          position = maxRotations;
        else if (homeRotations >= maxRotations - tolerance) 
          position = minRotations;
        else
          position = (minRotations + maxRotations) / 2;
      }
      m_Position.setPosition(position);
    }

    // Attempt calibrating once we have started moving
    if (active)
    {
      if (atLimit())
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
          (homeRotations <= minRotations + tolerance && request.Position <= getAngle())
          ||
          (homeRotations >= maxRotations - tolerance && request.Position >= getAngle())
        ) stop();
      }
      else // if not at limit
      {
        // When the sensor first *becomes* false, calibrate
        if (homeLastCycle && !calibrated)
        {
          calibrated = true;
          m_Position.setPosition(homeRotations);
          if (safeSpeedValid)
            {m_Position.getConfigurator().apply(motorConfig);}
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
    private final boolean invert;

    public DIOLimit(int limitIO, boolean invert)
    {
      io_Limit = new DigitalInput(limitIO);
      this.invert = invert;
    }

    @Override
    public boolean atLimit() 
      {return io_Limit.get() ^ invert;}
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

  private class SimLimit implements Limit 
  {
    @Override
    public boolean atLimit() 
      {return getAngle() <= homeRotations;}
  }
}

