package frc.robot.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.leds.patterns.*;

public class Block
{
  public final int length;

  private LEDPattern pattern;

  public Block(int length, LEDPattern pattern)
  {
    this.length = length;
    this.pattern = pattern;
  }

  public Block(int length)
    {this(length, Patterns.dummy);}

  public void setPattern(LEDPattern pattern)
    {this.pattern = pattern;}

  public static void setPatternMulti(LEDPattern pattern, Block... blocks)
    {for (var block : blocks) block.setPattern(pattern);}

  public void render(AddressableLEDBuffer stripBuffer, int start, double brightness)
  {
    pattern
      .atBrightness(Units.Value.of(brightness))
      .applyTo(stripBuffer.createView(start, start + length - 1));
  }
}