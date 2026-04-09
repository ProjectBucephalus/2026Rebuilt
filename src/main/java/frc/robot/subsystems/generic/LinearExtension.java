package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;

/** 
 * Generic subclass for a linear extension with a binary switch at the home position 
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class LinearExtension extends LimitedMotor 
{
  private final double metersPerRotation;

  /**
   * Creates generic linear extension system
   * @param motorCAN CAN-ID of extension motor
   * @param limitIO DIO-ID of home limit-sensor
   * @param minPosition Minimum position in meters
   * @param maxPosition Maximum position in meters
   * @param metersPerRotation Meters of extension per mechanism rotation
   * @param configs Motor configuration object, uses Slot1 if present when not calibrated
   */
  public LinearExtension(int motorCAN, int limitIO, double minPosition, double maxPosition, double homePosition, double metersPerRotation, TalonFXConfiguration configs)
  {
    super(motorCAN, limitIO, minPosition / metersPerRotation, maxPosition / metersPerRotation, homePosition / metersPerRotation, configs);
    this.metersPerRotation = metersPerRotation;
  } 

  /** @return Current physical position, in meters */
  @Logged(name = "Position Meters")
  public double getPosition()
    {return super.getAngle() * metersPerRotation;}

  /**
   * Sets the target point for the extension 
   * @param target meters
   */
  @Override
  public void setTarget(double target) 
    {super.setTarget(target / metersPerRotation);}

  /** @return Command to set the target to the maximum limit */
  @Override
  public Command extendCmd() 
    {return runOnce(() -> super.setTarget(maxRotations));}
  /** @return Command to set the target to the minimum limit */
  @Override
  public Command retractCmd() 
    {return runOnce(() -> super.setTarget(minRotations));}

  /** @param shiftSup A supplier for the amount to adjust the target by in meters */
  @Override
  public Command adjustTargetCmd(DoubleSupplier shiftSup) 
    {return super.adjustTargetCmd(() -> shiftSup.getAsDouble() / metersPerRotation);}
}
