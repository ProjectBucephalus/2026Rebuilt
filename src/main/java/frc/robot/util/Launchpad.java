// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Launchpad extends GenericHID
{
    private final String tableName = "LaunchPadColours";
    private final IntegerPublisher[] publishers = new IntegerPublisher[72];

    public enum PadColor
    {
        OFF(12),
        DIM_RED(13),
        MEDIUM_RED(14),
        FULL_RED(15),
        DIM_GREEN(28),
        DIM_AMBER(29),
        MEDIUM_ORANGE(30),
        FULL_ORANGE_RED(31),
        MEDIUM_GREEN(44),
        MEDIUM_YELLOW_GREEN(45),
        MEDIUM_AMBER(46),
        FULL_ORANGE(47),
        FULL_GREEN(60),
        FULL_YELLOW_GREEN(61),
        FULL_YELLOW(62),
        FULL_AMBER(63);

        public final int value;

        PadColor(int value) {
            this.value = value;
        }
    }

    public Launchpad(int port) {
        super(port);

        var ntInstance = NetworkTableInstance.getDefault();
        ntInstance.startServer();
        var table = ntInstance.getTable(tableName);
        for (int i = 0; i < 72; i++) {
            var topic = table.getIntegerTopic(Integer.toString(i));
            publishers[i] = topic.publish();
        }
    }

    /** btn should be in range [0..63]. Don't ask for -1, you'll make me sad :( */
    public Trigger getBtn(int btn) 
    {
        return new Trigger(() -> {
            if (btn >= 32)  
                return super.getRawButton(btn - 32);
            else if (btn < 16) 
                return super.getPOV(1) == btn;
            else if (btn < 24) 
                return super.getPOV(2) == btn - 16;
            else if (btn < 32)
                return super.getPOV(3) == btn - 24;
            else 
                return false;
        });
    }

    /** btn should be in range [0..7]. Don't ask for -1, you'll make me sad :( */
    public Trigger getModeBtn(int modeBtn) 
        {return new Trigger(() -> super.getPOV(0) == modeBtn);}

    public void setColour(int btn, PadColor colour) {
        publishers[btn].accept(colour.value);
    }
}
