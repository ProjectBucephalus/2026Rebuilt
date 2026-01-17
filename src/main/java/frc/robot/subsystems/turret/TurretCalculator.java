package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.Conversions;

import static frc.robot.constants.Constants.Turret.maxTurretAzimuth;

public class TurretCalculator 
{
  public static double normaliseAngle(double newAngle, double currentAngle)
  {
    double newAngleWrapped = Conversions.mod(newAngle, 360);
    double currentAngleWrapped = Conversions.mod(currentAngle, 360);

    double offset = MathUtil.inputModulus(newAngleWrapped - currentAngleWrapped, -180, 180);

    double targetAngle = currentAngleWrapped + offset;

    if (targetAngle > maxTurretAzimuth)
      {return targetAngle - 360;}
    else if (targetAngle < -maxTurretAzimuth)
      {return targetAngle + 360;}
    else 
      {return targetAngle;}
  }
}