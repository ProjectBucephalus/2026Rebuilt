package frc.robot.leds.patterns;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Conversions;

public class ChasePattern implements LEDPattern
{
  private final Color colour1;
  private final Color colour2;
  private final double period;

  private double lastSwap = 0;
  private int cycle = 0;
  private int step = 1;

  private int length1;
  private int frames;

  /**
   * Creates a pattern of rolling blocks of the given colours, cycling at the given frequency
   * @param colour1 Primary colour
   * @param length1 Number of LEDs in sequence to display primary colour
   * @param colour2 Secondary colour
   * @param length2 Number of LEDs in sequence to display secondary colour
   * @param frequency Full cycles per second, Hz, use negative value to reverse direction
   */
  public ChasePattern(Color colour1, int length1, Color colour2, int length2, double frequency)
  {
    this.colour1 = colour1;
    this.length1 = Math.max(length1, 1);
    this.colour2 = colour2;
    length2 = Math.max(length2, 1);
    frames = length1 + length2 - 1;
    
    if (frequency == 0)
    {
      period = 60;
      step = 0;
    }
    else
    {
      period  = 1.0 / Math.abs(frequency * (length1 + length2));
      step = frequency < 0 ? -1 : 1;
    }
  }

  /**
   * Creates a pattern that sets alternate LEDs to the given colours, swapping at the given frequency
   * @param colour1 Primary colour
   * @param colour2 Secondary colour
   * @param frequency Full cycles per second, Hz, use negative value to reverse direction
   */
  public ChasePattern(Color colour1, Color colour2, double frequency)
    {this(colour1, 1, colour2, 1, frequency);}
  
  /**
   * Creates a simple chase pattern of the given colour, with a single lit LED, cycling at the given frequency
   * @param colour Colour of lit LEDs
   * @param frames Number of LEDs in sequence, one of which will be lit, the rest dark
   * @param frequency Full cycles per second, Hz, use negative value to reverse direction
   */
  public ChasePattern(Color colour, int frames, double frequency)
    {this(colour, 1, Color.kBlack, frames-1, frequency);}

  /**
   * Creates a simple 3-cell chase pattern of the given colour, cycling at the given frequency
   * @param colour Colour of lit LEDs
   * @param frequency Full cycles per second, Hz, use negative value to reverse direction
   */
  public ChasePattern(Color colour, double frequency)
    {this(colour, 1, Color.kBlack, 2, frequency);}

  @Override
  public void applyTo(LEDReader reader, LEDWriter writer) 
  {
    if (Timer.getTimestamp() - lastSwap >= period)
    {
      lastSwap = Timer.getTimestamp();
      cycle = Conversions.wrap(cycle + step, 0, frames);
    }
      
    for (int i = 0; i < reader.getLength(); i++)
      writer.setLED(i, Conversions.wrap(cycle + i, 0, frames) < length1 ? colour1 : colour2);
  }
}
