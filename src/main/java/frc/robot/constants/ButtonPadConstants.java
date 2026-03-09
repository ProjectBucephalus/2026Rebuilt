package frc.robot.constants;

import frc.robot.util.Launchpad;
import frc.robot.util.Launchpad.DisplayGrid;
import frc.robot.util.Launchpad.PadColour;

/** Values for setting displays and binding controlls to the button pad */
public class ButtonPadConstants 
{
    private static final PadColour OF = PadColour.OFF;
    private static final PadColour DG = PadColour.DIM_GREEN;
    private static final PadColour MG = PadColour.MEDIUM_GREEN;
    private static final PadColour FG = PadColour.FULL_GREEN;
    private static final PadColour DR = PadColour.DIM_RED;
    private static final PadColour MR = PadColour.MEDIUM_RED;
    private static final PadColour FR = PadColour.FULL_RED;
    private static final PadColour DA = PadColour.DIM_AMBER;
    private static final PadColour MA = PadColour.MEDIUM_AMBER;
    private static final PadColour FA = PadColour.FULL_AMBER;
    private static final PadColour ML = PadColour.MEDIUM_YELLOW_GREEN;
    private static final PadColour FL = PadColour.FULL_YELLOW_GREEN;
    private static final PadColour FY = PadColour.FULL_YELLOW;
    private static final PadColour MO = PadColour.MEDIUM_ORANGE;
    private static final PadColour FO = PadColour.FULL_ORANGE;
    private static final PadColour FC = PadColour.FULL_ORANGE_RED;

    public static final DisplayGrid passPointMap = new DisplayGrid
    (
        new PadColour[]
        {
            MA, OF, MR, FO, FO, MG, OF, MA,
            MR, MR, MR, FO, FO, MG, MG, MG,
            OF, FY, OF, FY, FY, OF, FY, OF,
            DA, FA, DA, FA, FA, DA, FA, DA,
            OF, OF, OF, OF, OF, OF, OF, OF,
            OF, OF, OF, OF, OF, OF, OF, OF,
            OF, OF, OF, OF, OF, OF, OF, OF,
            OF, DA, DA, OF, DA, OF, OF, DA,

            FG, MR, OF, MG
        }
    );

    public static final DisplayGrid localisationMap = new DisplayGrid
    (
        new PadColour[]
        {
            MA, OF, MR, FO, FO, MG, OF, MA,
            MR, MR, MR, FO, FO, MG, MG, MG,
            OF, FY, OF, FY, FY, OF, FY, OF,
            DA, FA, DA, FA, FA, DA, FA, DA,
            DR, DR, DR, DR, DR, DR, DR, DR,
            DR, DR, DR, DR, DR, DR, DR, DR,
            DR, DR, DR, DR, DR, DR, DR, DR,
            DR, DA, DA, DR, DA, DR, DR, DA,

            MG, FR, OF, MG
        }
    );

    public static final DisplayGrid manualControlGrid = new DisplayGrid
    (
        new PadColour[]
        {
            MR, OF, MA, OF, OF, MA, OF, MG,
            MR, FO, MA, FO, FY, MA, FY, MG,
            MR, OF, OF, OF, OF, OF, OF, MG,
            MR, DA, FO, DA, DA, FO, DA, MG,
            OF, OF, OF, OF, OF, OF, OF, OF,
            FO, FO, FO, OF, OF, FO, FO, OF,
            FO, FY, FY, OF, OF, FY, FY, OF,
            OF, FO, FO, OF, OF, FY, FY, OF,

            MG, MR, OF, FG
        }
    );

    public static final DisplayGrid colourChart = new DisplayGrid
    (
        new PadColour[]
        {
            OF, OF, DG, DG, MG, MG, FG, FG,
            OF, OF, DG, DG, MG, MG, FG, FG,
            DR, DR, DA, DA, ML, ML, FL, FL,
            DR, DR, DA, DA, ML, ML, FL, FL,
            MR, MR, MO, MO, MA, MA, FY, FY,
            MR, MR, MO, MO, MA, MA, FY, FY,
            FR, FR, FC, FC, FO, FO, FA, FA,
            FR, FR, FC, FC, FO, FO, FA, FA
        }
    );
}