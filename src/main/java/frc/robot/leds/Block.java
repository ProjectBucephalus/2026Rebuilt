package frc.robot.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.leds.patterns.*;
import frc.robot.util.PBDash;

public class Block
{
  public final int length;

  private LEDPattern pattern;

  private PBDash.Key<String[]> ntAddress = null;
  private AddressableLEDBuffer ntBuffer = new AddressableLEDBuffer(4);

  /**
   * Creates a new LED Block
   * @param length Number of LEDs in the block
   * @param pattern Initial pattern for the block to display
   */
  public Block(int length, LEDPattern pattern)
  {
    this.length = length;
    this.pattern = pattern;
  }

  /**
   * Creates a new LED Block with empty default pattern
   * @param length Number of LEDs in the block
   */
  public Block(int length)
    {this(length, Patterns.dummy);}

  /** @param pattern Colour pattern to display on the block */
  public void setPattern(LEDPattern pattern)
    {this.pattern = pattern;}

  /**
   * Sets all blocks in an array to display the same pattern
   * @param pattern Colour pattern to display on each block
   * @param blocks List of LED blocks to apply pattern to
   */
  public static void setPatternMulti(LEDPattern pattern, Block... blocks)
    {for (var block : blocks) block.setPattern(pattern);}

  /**
   * When this block renders, the first 4 colours will be sent to the network to be displayed on the dashboard
   * @param ntAddress String Array key to publish the colour values
   */
  public void setNetworkTableAddress(PBDash.Key<String[]> ntAddress)
    {this.ntAddress = ntAddress;}

  /**
   * When the first block in this set renders, the first 4 colours will be sent to the network to be displayed on the dashboard
   * @param ntAddress String Array key to publish the colour values
   * @param blocks List of LED blocks which share a pattern
   */
  public static void setNTAddressMulti(PBDash.Key<String[]> ntAddress, Block... blocks)
    {if (blocks.length != 0) blocks[0].setNetworkTableAddress(ntAddress);}

  /**
   * Renders this block's pattern to the provided LED strip, and if set, to the network table
   * @param stripBuffer LED strip buffer to render to
   * @param start Index of the first LED in this block within the full strip
   * @param brightness Multiplier to apply to all channels to modify brightness, [0..1]
   */
  public void render(AddressableLEDBuffer stripBuffer, int start, double brightness)
  {
    pattern
      .atBrightness(Units.Value.of(brightness))
      .applyTo(stripBuffer.createView(start, start + length - 1));

    if (ntAddress != null)
    {
      pattern.applyTo(ntBuffer);
      
      // getLED is non-trivial and entails a memory allocation
      var ledZero = ntBuffer.getLED(0);
      if 
      (
        ledZero.equals(ntBuffer.getLED(1))
        && ledZero.equals(ntBuffer.getLED(2))
        && ledZero.equals(ntBuffer.getLED(3))
      )
        ntAddress.put(new String[] {ledZero.toHexString()});
      else
        ntAddress.put
        (
          new String[]
          {
            ledZero.toHexString(),
            ntBuffer.getLED(1).toHexString(),
            ntBuffer.getLED(2).toHexString(),
            ntBuffer.getLED(3).toHexString()
          }
        );
    }
  }
}