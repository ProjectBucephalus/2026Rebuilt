package frc.robot.leds.Patterns;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;

public record SolidPattern(Color colour) implements Pattern
{
  @Override
  public <T extends LEDWriter & LEDReader> void render(T buffer)
    {LEDPattern.solid(colour).applyTo(buffer);}
}
