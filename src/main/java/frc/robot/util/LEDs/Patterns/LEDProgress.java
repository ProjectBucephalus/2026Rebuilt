package frc.robot.util.LEDs.Patterns;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDProgress extends LEDPatternObject
{
    private Color back;
    private Supplier<Double> progress;

    public LEDProgress (Supplier<Double> progressValue, Color frontColour, Color backColor)
    {
        super(frontColour);
        progress = progressValue;
        back = backColor;
    }

    public LEDPatternObject setBackColour(Color newBack)
    {
        back = newBack;
        return this;
    }

    public Color getBackColour()
    {
        return back;
    }

    @Override
    public AddressableLEDBuffer Render(AddressableLEDBuffer buffer)
    {
        LEDPattern.steps(Map.of(0,colour,progress.get(),back)).applyTo(buffer);
        return buffer;
    }
}
