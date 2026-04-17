package frc.robot.leds.patterns;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.LEDPattern;

public class Patterns 
{
  public static final LEDPattern dummy = (reader, writer) -> {};

  public static final LEDPattern conditional(LEDPattern inner, BooleanSupplier cond) 
    {return (reader, writer) -> { if (cond.getAsBoolean()) inner.applyTo(reader, writer); };}
}
