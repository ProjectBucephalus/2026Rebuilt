package frc.robot.leds.patterns;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class Patterns 
{
  public static final LEDPattern dummy = (reader, writer) -> {};

  public static final LEDPattern conditional(BooleanSupplier cond, LEDPattern truePattern)
    {return conditional(cond, truePattern, dummy);}
  
  public static final LEDPattern conditional(BooleanSupplier cond, LEDPattern truePattern, LEDPattern falsePattern) 
  {
    return (reader, writer) -> 
    {
      if (cond.getAsBoolean()) 
        truePattern.applyTo(reader, writer); 
      else
        falsePattern.applyTo(reader, writer);
    };
  }

  public static final LEDPattern supplied(Supplier<Color> colourSup)
    {return (reader, writer) -> LEDPattern.solid(colourSup.get()).applyTo(reader, writer);}
}
