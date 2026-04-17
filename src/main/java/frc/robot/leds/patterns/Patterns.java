package frc.robot.leds.patterns;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class Patterns 
{
  public static final LEDPattern dummy = (reader, writer) -> {};

  public static final LEDPattern conditional(LEDPattern inner, BooleanSupplier cond) 
    {return (reader, writer) -> { if (cond.getAsBoolean()) inner.applyTo(reader, writer); };}

  public static final LEDPattern supplied(Supplier<Color> colourSup)
    {return (reader, writer) -> LEDPattern.solid(colourSup.get()).applyTo(reader, writer);}
}
