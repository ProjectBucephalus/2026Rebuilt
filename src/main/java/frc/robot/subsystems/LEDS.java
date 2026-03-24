package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Constants.LEDConstants;
import frc.robot.leds.Sections.LEDSection;

import java.util.ArrayList;
import java.util.Comparator;

public class LEDS extends SubsystemBase
{
    AddressableLED LEDStrip;
    AddressableLEDBuffer LEDBuffer;
    ArrayList<LEDSection> sectionList = new ArrayList<LEDSection>();

    public LEDS()
    {
        LEDStrip = new AddressableLED(IDConstants.LEDPWDPort);
        LEDBuffer = new AddressableLEDBuffer(LEDConstants.LEDStripLen);
        LEDStrip.setLength(LEDConstants.LEDStripLen);
        LEDStrip.start();
    }

    public void registerSection(LEDSection newSection)
    {
        sectionList.add(newSection);
    }

    public void removeSection(int index)
    {
        sectionList.remove(index);
    }

    public void removeSection(String name)
    {
        sectionList.removeIf(a -> a.getName() == name);
    }

    //TODO: list section method and remove section by name method (section names?) 

    @Override
    public void periodic()
    {
        LEDPattern.solid(Color.kBlack).applyTo(LEDBuffer);
        sectionList.sort(Comparator.comparing(LEDSection::getPriority));
        for (LEDSection section : sectionList)
        {
            if (section.getPriority() > 0)
            {
                LEDBuffer = section.render(LEDBuffer);
            }
        }
        LEDStrip.setData(LEDBuffer);
    }
}
