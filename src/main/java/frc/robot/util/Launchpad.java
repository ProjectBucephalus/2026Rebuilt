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
  private static final String tableName = "LaunchPadColours";

  private final Trigger no = new Trigger(() -> false);
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

    public static final PadColour OF = PadColour.OFF;
    public static final PadColour DG = PadColour.DIM_GREEN;
    public static final PadColour MG = PadColour.MEDIUM_GREEN;
    public static final PadColour FG = PadColour.FULL_GREEN;
    public static final PadColour DR = PadColour.DIM_RED;
    public static final PadColour MR = PadColour.MEDIUM_RED;
    public static final PadColour FR = PadColour.FULL_RED;
    public static final PadColour DA = PadColour.DIM_AMBER;
    public static final PadColour MA = PadColour.MEDIUM_AMBER;
    public static final PadColour FA = PadColour.FULL_AMBER;
    public static final PadColour ML = PadColour.MEDIUM_YELLOW_GREEN;
    public static final PadColour FL = PadColour.FULL_YELLOW_GREEN;
    public static final PadColour FY = PadColour.FULL_YELLOW;
    public static final PadColour MO = PadColour.MEDIUM_ORANGE;
    public static final PadColour FO = PadColour.FULL_ORANGE;
    public static final PadColour FC = PadColour.FULL_ORANGE_RED;
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public Launchpad(int port) 
  {
    // Creates generic HID
    controllerOne = new CommandGenericHID(port);
    controllerTwo = new CommandGenericHID(port + 1);

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
   * Flags multiple errors if the virtual controllers are ordered incorrectly in driverstation
   * @return {@code true} iff both controllers are plugged in correctly
   */
  public boolean validate()
  {
    if (controllerOne.getHID().getPOVCount() != 2 || controllerTwo.getHID().getPOVCount() != 3)
    {
      setColourSpan(PadColour.FULL_RED, 0, 31);
      setColourSpan(PadColour.FULL_ORANGE, 32, 63);
      return false;
    }

    return true;
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

  /**
   * Pushes a full list of colour values to be dsiplayed, starting from 0
   * <p> intended for full display refresh
   * @param display List of Colour values
   */
  public void setDisplayGrid(DisplayGrid display)
  {
    for (int i = 0; i < display.grid.length; i++)
      {publishers[i].accept(display.grid[i].value);}
  }

  
  public record DisplayGrid(PadColour... grid){}

  /**
   * Converts integer array to PadColour array, for easier setting
   * @param grid up to 8x8 Colour reference grid:
   * <li> 0 -> Off
   * <li> 1,4,7 -> Red
   * <li> 2,5,8 -> Amber
   * <li> 3,6,9 -> Green
   * @return
   */
  public static DisplayGrid generateDisplayGrid(int... grid)
  {
    PadColour[] processingGrid = new PadColour[grid.length];
    for (int i = 0; i < grid.length; i++)
      switch (grid[i]) 
      {
        case 1:
          processingGrid[i] = PadColour.DIM_RED;
          break;
        case 4:
          processingGrid[i] = PadColour.MEDIUM_RED;
          break;
        case 7:
          processingGrid[i] = PadColour.FULL_RED;
          break;
        case 2:
          processingGrid[i] = PadColour.DIM_AMBER;
          break;
        case 5:
          processingGrid[i] = PadColour.MEDIUM_AMBER;
          break;
        case 8:
          processingGrid[i] = PadColour.FULL_AMBER;
          break;
        case 3:
          processingGrid[i] = PadColour.DIM_GREEN;
          break;
        case 6:
          processingGrid[i] = PadColour.MEDIUM_GREEN;
          break;
        case 9:
          processingGrid[i] = PadColour.FULL_GREEN;
          break;
        case 0:
        default:
          processingGrid[i] = PadColour.OFF;
          break;
      }

    return new DisplayGrid(processingGrid);
  }

  //   A B C D E F G H  M
  // 1 [][][][][][][][] ()
  // 2 [][][][][][][][] ()
  // 3 [][][][][][][][] ()
  // 4 [][][][][][][][] ()
  // 5 [][][][][][][][] ()
  // 6 [][][][][][][][] ()
  // 7 [][][][][][][][] ()
  // 8 [][][][][][][][] ()

  public Trigger A1() {return getBtn(0);}
  public Trigger B1() {return getBtn(1);}
  public Trigger C1() {return getBtn(2);}
  public Trigger D1() {return getBtn(3);}
  public Trigger E1() {return getBtn(4);}
  public Trigger F1() {return getBtn(5);}
  public Trigger G1() {return getBtn(6);}
  public Trigger H1() {return getBtn(7);}
  public Trigger A2() {return getBtn(8);}
  public Trigger B2() {return getBtn(9);}
  public Trigger C2() {return getBtn(10);}
  public Trigger D2() {return getBtn(11);}
  public Trigger E2() {return getBtn(12);}
  public Trigger F2() {return getBtn(13);}
  public Trigger G2() {return getBtn(14);}
  public Trigger H2() {return getBtn(15);}
  public Trigger A3() {return getBtn(16);}
  public Trigger B3() {return getBtn(17);}
  public Trigger C3() {return getBtn(18);}
  public Trigger D3() {return getBtn(19);}
  public Trigger E3() {return getBtn(20);}
  public Trigger F3() {return getBtn(21);}
  public Trigger G3() {return getBtn(22);}
  public Trigger H3() {return getBtn(23);}
  public Trigger A4() {return getBtn(24);}
  public Trigger B4() {return getBtn(25);}
  public Trigger C4() {return getBtn(26);}
  public Trigger D4() {return getBtn(27);}
  public Trigger E4() {return getBtn(28);}
  public Trigger F4() {return getBtn(29);}
  public Trigger G4() {return getBtn(30);}
  public Trigger H4() {return getBtn(31);}
  public Trigger A5() {return getBtn(32);}
  public Trigger B5() {return getBtn(33);}
  public Trigger C5() {return getBtn(34);}
  public Trigger D5() {return getBtn(35);}
  public Trigger E5() {return getBtn(36);}
  public Trigger F5() {return getBtn(37);}
  public Trigger G5() {return getBtn(38);}
  public Trigger H5() {return getBtn(39);}
  public Trigger A6() {return getBtn(40);}
  public Trigger B6() {return getBtn(41);}
  public Trigger C6() {return getBtn(42);}
  public Trigger D6() {return getBtn(43);}
  public Trigger E6() {return getBtn(44);}
  public Trigger F6() {return getBtn(45);}
  public Trigger G6() {return getBtn(46);}
  public Trigger H6() {return getBtn(47);}
  public Trigger A7() {return getBtn(48);}
  public Trigger B7() {return getBtn(49);}
  public Trigger C7() {return getBtn(50);}
  public Trigger D7() {return getBtn(51);}
  public Trigger E7() {return getBtn(52);}
  public Trigger F7() {return getBtn(53);}
  public Trigger G7() {return getBtn(54);}
  public Trigger H7() {return getBtn(55);}
  public Trigger A8() {return getBtn(56);}
  public Trigger B8() {return getBtn(57);}
  public Trigger C8() {return getBtn(58);}
  public Trigger D8() {return getBtn(59);}
  public Trigger E8() {return getBtn(60);}
  public Trigger F8() {return getBtn(61);}
  public Trigger G8() {return getBtn(62);}
  public Trigger H8() {return getBtn(63);}

  public Trigger M1() {return getModeBtn(0);}
  public Trigger M2() {return getModeBtn(1);}
  public Trigger M3() {return getModeBtn(2);}
  public Trigger M4() {return getModeBtn(3);}
  public Trigger M5() {return getModeBtn(4);}
  public Trigger M6() {return getModeBtn(5);}
  public Trigger M7() {return getModeBtn(6);}
  public Trigger M8() {return getModeBtn(7);}
}
