package frc.robot.util.LEDs.Sections;

import frc.robot.util.LEDs.Patterns.*;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSection {

    private String name;
    private int start;
    private int width;
    private LEDPatternObject Pattern;
    private boolean border = false;
    private int priority = 0;

    private LEDSection (String name, int start, int width, LEDPatternObject Pattern)
    {
        this.name = name;
        this.start = start;
        this.width = width;
        this.Pattern = Pattern;
    }

    private LEDSection (int start, int width, LEDPatternObject Pattern)
    {
        this.name = "LEDSection";
        this.start = start;
        this.width = width;
        this.Pattern = Pattern;
    }

    public LEDSection setBorder(boolean newBorder)
    {
        border = newBorder;
        return this;
    }

    private int calcStart()
    {
        return start;
    }

    public LEDSection setPriority(int newPriority)
    {
        priority = newPriority;
        return this;
    }

    public int getPriority()
    {
        return priority;
    }

    public String getName()
    {
        return name;
    }

    public AddressableLEDBuffer render (AddressableLEDBuffer stripBuffer)
    {
        AddressableLEDBuffer sectionBuffer = new AddressableLEDBuffer(width);
        sectionBuffer = Pattern.Render(sectionBuffer);
        int renderStart = calcStart();
        if (border)
        {
            sectionBuffer.setRGB(0,0,0,1);
            sectionBuffer.setRGB(width-1, 0, 0, 1);
        }
        for (int c = 0; c < width; c++)
        {
            int stripIndex = renderStart + c;
            if (stripIndex > stripBuffer.getLength())
            {
                stripIndex -= stripBuffer.getLength();
            }
            Color sectionIndexColor = sectionBuffer.getLED(c);
            if (!(sectionIndexColor.equals(new Color(0,0,0))))
            stripBuffer.setLED(stripIndex, sectionIndexColor);
        }
        return stripBuffer;
    }
}
