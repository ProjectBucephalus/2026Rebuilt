package frc.robot.util.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.FieldUtils;
import frc.robot.util.MatchTime;

public class LEDAlarmPattern extends LEDPatternObject{

    double aTime;
    double aDuration;
    double aFrequency;
    int aStyle; // 0 = solid, 1 = flashing , 2 = crawling

    public LEDAlarmPattern(double alarmTime, double AlarmDuration, Color colour)
    {
        super(colour);
        aTime = alarmTime;
        aDuration = AlarmDuration;
        aStyle = 1;
        aFrequency = 0.25;
    }

    public LEDAlarmPattern(double alarmTime, double AlarmDuration, int alarmStyle, Color colour)
    {
        super(colour);
        aTime = alarmTime;
        aDuration = AlarmDuration;
        aStyle = alarmStyle;
        aFrequency = 0.25;
    }

    public LEDAlarmPattern(double alarmTime, double alarmDuration, double alarmFrequency, int alarmStyle, Color colour)
    {
        super(colour);
        aTime = alarmTime;
        aDuration = alarmDuration;
        aStyle = alarmStyle;
        aFrequency = alarmFrequency;
    }
    

    @Override
    public AddressableLEDBuffer Render(AddressableLEDBuffer buffer)
    {
        LEDPattern.solid(Color.kBlack).applyTo(buffer);
        double curTime = MatchTime.getGameTimeElapsed();
        if ((curTime > aTime) && (curTime < (aTime + aDuration)))
        {
            switch (aStyle) {
                case 0:
                    LEDPattern.solid(colour).applyTo(buffer);
                    break;

                case 1:
                    if ((int)(Math.floor((curTime-aTime)/aFrequency))%2 == 0)
                    {
                        LEDPattern.solid(colour).applyTo(buffer);
                    }
                    break;

                case 2:
                    int o = (int)(Math.floor((curTime-aTime)/aFrequency))%2;
                    for (int c = o; c < buffer.getLength(); c += 2)
                    {
                        buffer.setLED(c, colour);
                    }
                    break;
            
                default:
                    break;
            } 
        }
        return buffer;
    }
}
