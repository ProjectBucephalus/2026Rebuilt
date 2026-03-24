package frc.robot.controlTransmutation;
import edu.wpi.first.math.geometry.Translation2d;

/** 
 * Standard interface for input transformation functions 
 * @author 5985
 */
public interface InputTransmuter
{
  /**
   * Takes in, transmutes, and returns a joystick input
   * @param controlInput Origingal joystick input [-1..1],[-1..1]
   * @return Transmuted joystick output [-1..1],[-1..1]
   */
  public default Translation2d process(Translation2d controlInput) 
    {return controlInput;}
}
