package frc.robot.util.controlTransmutation;
import edu.wpi.first.math.geometry.Translation2d;

/** Standard interface for input transformation functions */
public abstract class InputTransmuter
{
  /**
   * Takes in, transmutes, and returns a joystick input
   * @param controlInput Origingal joystick input [-1..1],[-1..1]
   * @return Transmuted joystick output [-1..1],[-1..1]
   */
  public Translation2d process(Translation2d controlInput)
    {return controlInput;}
}
