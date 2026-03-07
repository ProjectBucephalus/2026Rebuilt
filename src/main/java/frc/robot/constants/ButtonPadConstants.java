package frc.robot.constants;

import frc.robot.util.Launchpad;
import frc.robot.util.Launchpad.DisplayGrid;

/** Values for setting displays and binding controlls to the button pad */
public class ButtonPadConstants 
{
    public static final DisplayGrid testGrid = Launchpad.generateDisplayGrid
    (
        0, 1, 2, 3, 4, 5, 6, 7,
        8, 9, 0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 0, 1, 2, 3,
        4, 5, 6, 7, 8, 9, 0, 1,
        2, 3, 4, 5, 6, 7, 8, 9,
        0, 1, 2, 3, 4, 5, 6, 7,
        8, 9, 0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 0, 1, 2, 3
    );

    public static final DisplayGrid passPointMap = Launchpad.generateDisplayGrid
    (
        6, 6, 0, 5, 5, 0, 4, 6,
        5, 0, 5, 4, 4, 5, 4, 6,
        4, 6, 0, 5, 5, 0, 4, 6,
        8, 7, 8, 9, 9, 8, 7, 8,
        3, 3, 3, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 3, 3,
        3, 9, 9, 3, 8, 3, 3, 9,

        9, 1, 0, 0, 0, 0, 0, 0
    );

    public static final DisplayGrid localisationMap = Launchpad.generateDisplayGrid
    (
        6, 6, 0, 5, 5, 0, 4, 6,
        5, 0, 5, 4, 4, 5, 4, 6,
        4, 6, 0, 5, 5, 0, 4, 6,
        8, 7, 8, 9, 9, 8, 7, 8,
        1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1,
        1, 9, 9, 1, 8, 1, 1, 9,

        3, 7, 0, 0, 0, 0, 0, 0
    );
}