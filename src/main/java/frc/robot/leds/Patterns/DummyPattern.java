package frc.robot.leds.Patterns;

import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;

public class DummyPattern implements Pattern 
{
  @Override
  public <T extends LEDWriter & LEDReader> void render(T buffer) 
    {/* This class acts as a dummy, no-op patten for use as a default */}
}
