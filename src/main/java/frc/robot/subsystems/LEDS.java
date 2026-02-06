package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Constants.LEDConstants;
import frc.robot.util.LEDs.Sections.LEDSection;

import java.util.ArrayList;
import java.util.Comparator;

public class LEDS extends SubsystemBase
{
    AddressableLED LEDStrip;
    AddressableLEDBuffer LEDBuffer;
    final LEDPattern patternBlack = LEDPattern.solid(Color.kBlack); //Useful to wipe the buffer before each render pass
    ArrayList<LEDSection> sectionList = new ArrayList<LEDSection>();

    public LEDS()
    {
        LEDStrip = new AddressableLED(IDConstants.LEDPWDPort);
        LEDBuffer = new AddressableLEDBuffer(LEDConstants.LEDStripLen);
        LEDStrip.setLength(LEDConstants.LEDStripLen);
        LEDStrip.start();
    }

    //TODO: register section method and remove section method (names?)

    @Override
    public void periodic()
    {
        patternBlack.applyTo(LEDBuffer);
        sectionList.sort(Comparator.comparing(LEDSection::getPriority));
        for (LEDSection section : sectionList)
        {
            LEDBuffer = section.render(LEDBuffer);
        }
        LEDStrip.setData(LEDBuffer);
    }
}
