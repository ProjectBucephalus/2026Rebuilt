package frc.robot.subsystems.generic;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * Generic subclass for a linear extension with a binary switch at the home position 
 * @author 5985
 */
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
  public LinearExtension(int motorCAN, int limitIO, double minPosition, double maxPosition, double metersPerRotation, TalonFXConfiguration configs)
  {
    super(motorCAN, limitIO, minPosition / metersPerRotation, maxPosition / metersPerRotation, configs);
    this.metersPerRotation = metersPerRotation;
  } 

  /**
   * Creates a command to set the target point for the extension <p>
   * NOTE: The provided value is only evaluated when the command is created
   * @param targetPosition meters
   * @return the Command
   */
  @Override
  public Command setTargetCommand(double targetPosition) 
    {return super.setTargetCommand(targetPosition / metersPerRotation);}
}
