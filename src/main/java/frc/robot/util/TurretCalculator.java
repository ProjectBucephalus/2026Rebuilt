package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public class TurretCalculator 
{
    private static double maxAbsPos = Constants.Turret.maxTurretAzimuth;
    
    private static double turnBackThreshold = Constants.Turret.turnBackThreshold;


    public static double goToAngle(double newAngle, double currentAngle)
    {
    newAngle = Conversions.mod(newAngle, 360);
    double offset = MathUtil.inputModulus(newAngle -Conversions.mod(currentAngle, 360), -180, 180);

    if (Math.abs(offset) >= turnBackThreshold)
    {
    double reverseOffset = offset - Math.copySign(360, offset);

    if (Math.abs(currentAngle + offset) > Math.abs(currentAngle + reverseOffset))
    {return (currentAngle + reverseOffset);}
    else
    {return (currentAngle + offset);}
    }
    else if (currentAngle + offset > maxAbsPos)
        {return (currentAngle + offset - 360);}

    else if (currentAngle + offset > -maxAbsPos)
        {return (currentAngle + offset + 360);}

    else 
        {return (currentAngle + offset);}

    }


}