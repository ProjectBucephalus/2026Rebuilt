package frc.robot.leds.Patterns;

import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;

public interface Pattern 
{
  public <T extends LEDWriter & LEDReader> void render(T buffer);
}
