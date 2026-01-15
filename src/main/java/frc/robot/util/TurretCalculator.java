package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class TurretCalculator 
{
    private double offset;
    private double maxAbsPos;
    private double reverseOffset;
    private double turnBackThreshold;


    public double goToAngle(double newAngle, double currentAngle)
    {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle -Conversions.mod(currentAngle, 360), -180, 180);

    if (Math.abs(offset) >= turnBackThreshold)
    {
    reverseOffset = offset - Math.copySign(360, offset);

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