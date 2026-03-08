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

    public static final DisplayGrid testGrid = new DisplayGrid
    (
        new PadColour[]
        {
            OF, DR, DA, DG, MR, MA, MG, FR,
            FA, FG, OF, DR, DA, DG, MR, MA,
            MG, FR, FA, FG, OF, DR, DA, DG,
            MR, MA, MG, FR, FA, FG, OF, DR,
            DA, DG, MR, MA, MG, FR, FA, FG,
            OF, DR, DA, DG, MR, MA, MG, FR,
            FA, FG, OF, DR, DA, DG, MR, MA,
            MG, FR, FA, FG, OF, DR, DA, DG
        }
    );

    public static final DisplayGrid passPointMap = new DisplayGrid
    (
        new PadColour[]
        {
            MG, MG, OF, MA, MA, OF, MR, MG,
            MA, OF, MA, MR, MR, MA, MR, MG,
            MR, MG, OF, MA, MA, OF, MR, MG,
            FA, FR, FA, FG, FG, FA, FR, FA,
            DG, DG, DG, DG, DG, DG, DG, DG,
            DG, DG, DG, DG, DG, DG, DG, DG,
            DG, DG, DG, DG, DG, DG, DG, DG,
            DG, FG, FG, DG, FA, DG, DG, FG,

            FG, DR, OF, OF, OF, OF, OF, OF
        }
    );

    public static final DisplayGrid localisationMap = new DisplayGrid
    (
        new PadColour[]
        {
            MG, MG, OF, MA, MA, OF, MR, MG,
            MA, OF, MA, MR, MR, MA, MR, MG,
            MR, MG, OF, MA, MA, OF, MR, MG,
            FA, FR, FA, FG, FG, FA, FR, FA,
            DR, DR, DR, DR, DR, DR, DR, DR,
            DR, DR, DR, DR, DR, DR, DR, DR,
            DR, DR, DR, DR, DR, DR, DR, DR,
            DR, FG, FG, DR, FA, DR, DR, FG,

            DG, FR, OF, OF, OF, OF, OF, OF
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