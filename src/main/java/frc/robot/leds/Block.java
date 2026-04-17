package frc.robot.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.leds.Patterns.*;

public class Block
{
  public final int length;

  private Pattern pattern;

  public Block(int length, Pattern pattern)
  {
    this.length = length;
    this.pattern = pattern;
  }

  public Block(int length)
    {this(length, new DummyPattern());}

  public void setPattern(Pattern pattern)
    {this.pattern = pattern;}

  public static void setPatternMulti(Pattern pattern, Block... blocks)
    {for (var block : blocks) block.setPattern(pattern);}

  public void render(AddressableLEDBuffer stripBuffer, int start)
    {pattern.render(stripBuffer.createView(start, start + length - 1));}
}