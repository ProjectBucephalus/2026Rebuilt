package frc.robot.util;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * Interface for using a "novation LaunchPad S" as a HID controller and LED display <p>
 * Requires <a href="https://github.com/ProjectBucephalus/Launchpad-VJoy/releases/tag/main">an external program</a> to be run on the driverstation to convert between MIDI, HID, and NetworkTables
 * @author 5985
 */
public class Launchpad
{
  private final Trigger no = new Trigger(() -> false);

  private final String tableName = "LaunchPadColours";
  private final IntegerPublisher[] publishers = new IntegerPublisher[72];

  private final CommandGenericHID controllerOne;
  private final CommandGenericHID controllerTwo;

  /** 
   * Each button has a Red and Green LED with 4 levels, giving a set of 16 available colours
   */
  public enum PadColour
  {
    OFF(12),
    DIM_RED(13),
    MEDIUM_RED(14),
    FULL_RED(15),
    DIM_GREEN(28),
    DIM_AMBER(29),
    MEDIUM_ORANGE(30),
    FULL_ORANGE_RED(31),
    MEDIUM_GREEN(44),
    MEDIUM_YELLOW_GREEN(45),
    MEDIUM_AMBER(46),
    FULL_ORANGE(47),
    FULL_GREEN(60),
    FULL_YELLOW_GREEN(61),
    FULL_YELLOW(62),
    FULL_AMBER(63);

    public final int value;

    PadColour(int value) 
      {this.value = value;}
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public Launchpad(int port) 
  {
    // Creates generic HID
    CommandGenericHID tempA = new CommandGenericHID(port);
    CommandGenericHID tempB = new CommandGenericHID(port + 1);

    if (tempA.getHID().getPOVCount() == 2 && tempB.getHID().getPOVCount() == 3) 
    {
      controllerOne = tempA;
      controllerTwo = tempB;
    }
    else 
    {
      controllerOne = tempB;
      controllerTwo = tempA;
      if (!(tempA.getHID().getPOVCount() == 3 && tempB.getHID().getPOVCount() == 2))
        PBDash.LAUNCHPAD_GOOD.put(false);
    }

    // Accesses network tables and creates a table to send colour data over
    var ntInstance = NetworkTableInstance.getDefault();
    ntInstance.startServer();
    var table = ntInstance.getTable(tableName);

    // Create table value for each button
    for (int i = 0; i < 72; i++)
    {
      var topic = table.getIntegerTopic(Integer.toString(i));
      topic.setPersistent(false);
      publishers[i] = topic.publish();
      publishers[i].accept(PadColour.DIM_AMBER.value);
    }
  }

  /**
   * Creates a trigger linked to a button on the Launchpad controller
   * @param btn Button index, [0..63] normal reading order of the square buttons
   * @return Trigger linked to button
   */
  public Trigger getBtn(int btn) 
  {
    if (btn < 0 || btn >= 72)
      return no;
    else if (btn < 32) 
      return controllerOne.button(btn + 1);
    else if (btn < 64)
      return controllerTwo.button(btn - 31);
    else 
      return getModeBtn(btn - 64);
  }

  /**
   * Creates a trigger linked to a round button on the sidebar of the Launchpad controller <p>
   * Due to limitations in the controller interface these form a mutually exclusive set,
   * such that pressing a second button releases the first, etc.
   * @param modeBtn Button index, [0..7] from the top
   * @return Trigger linked to button
   */
  public Trigger getModeBtn(int modeBtn) 
  {
    if (modeBtn < 0 || modeBtn > 7)
      return no;        
    else
      return controllerOne.pov(modeBtn);
  }

  /**
   * Sets the colour value displayed on one or more buttons
   * @param colour Predefined colour value to set all given buttons to
   * @param buttons list of button indexes, [0..71] 
   * <ul>
   * <li> [0..63] normal reading order of square buttons
   * <li> [64..71] top-down of round sidebar buttons
   * </ul>
   */
  public void setColour(PadColour colour, int... buttons)
  {
    for (int btn : buttons)
      if (btn >= 0 && btn < 72)
        publishers[btn].accept(colour.value);
  }

  /**
   * Sets the colour value displayed on a continious sequence of buttons
   * @param colour Predefined colour value to set all given buttons to
   * @param start Index of first button, [0..71] as per setColour()
   * @param end Index of last button, [0..71] as per setColour()
   */
  public void setColourSpan(PadColour colour, int start, int end)
  {
    start = Conversions.clamp(start, 0, 71);
    end = Conversions.clamp(end, start, 71);
    for (int btn = start; btn <= end; btn++)
      publishers[btn].accept(colour.value);
  }
}
