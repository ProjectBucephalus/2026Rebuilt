package frc.robot.leds.Patterns;

import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class AlternatingPattern implements Pattern
{
  private final Color colour1;
  private final Color colour2;
  private final double frequency;

  private double lastSwap = 0;
  private boolean evenCycle = false;

  public AlternatingPattern(Color colour1, Color colour2, double frequency)
  {
    this.colour1 = colour1;
    this.colour2 = colour2;
    this.frequency = frequency;
  }

  @Override
  public <T extends LEDWriter & LEDReader> void render(T buffer) 
  {
    if (Timer.getTimestamp() - lastSwap >= frequency)
    {
      lastSwap = Timer.getTimestamp();
      evenCycle = !evenCycle;
    }
      
    for (int i = 0; i < buffer.getLength(); i++)
      buffer.setLED(i, (i % 2 == 0) == evenCycle ? colour1 : colour2);
  }
}
