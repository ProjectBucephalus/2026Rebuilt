package frc.robot.util.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDPatternObject
{

    private Color colour;

    public LEDPatternObject (Color colour)
    {
        this.colour = colour;
    }

    public AddressableLEDBuffer Render(AddressableLEDBuffer buffer)
    {
        LEDPattern.solid(colour).applyTo(buffer);
        return buffer;
    }

    public LEDPatternObject setColour(Color newColour)
    {
        colour = newColour;
        return this;
    }

    public Color getColour()
    {
        return colour;
    }

}
