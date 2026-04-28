package frc.robot.leds.patterns;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

@Deprecated(forRemoval =  true)
public class AlternatingPattern implements LEDPattern
{
  private final Color colour1;
  private final Color colour2;
  private final double period;

  private double lastSwap = 0;
  private boolean evenCycle = true;

  /**
   * Creates a pattern that sets alternate LEDs to the given colours, swapping at the given frequency
   * @param colour1 Colour of Even LEDs on first cycle
   * @param colour2 Colour of Odd LEDs on first cycle
   * @param frequency Cycles per second, Hz
   */
  public AlternatingPattern(Color colour1, Color colour2, double frequency)
  {
    this.colour1 = colour1;
    this.colour2 = colour2;
    this.period  = 1.0 / frequency;
  }

  /**
   * Creates a pattern that sets alternate LEDs to the given colour and black, swapping at the given frequency
   * @param colour Colour of Even LEDs on first cycle
   * @param frequency Cycles per second, Hz
   */
  public AlternatingPattern(Color colour, double frequency)
    {this(colour, Color.kBlack, frequency);}

  @Override
  public void applyTo(LEDReader reader, LEDWriter writer) 
  {
    if (Timer.getTimestamp() - lastSwap >= period)
    {
      lastSwap = Timer.getTimestamp();
      evenCycle = !evenCycle;
    }
      
    for (int i = 0; i < reader.getLength(); i++)
      writer.setLED(i, (i % 2 == 0) == evenCycle ? colour1 : colour2);
  }
}
