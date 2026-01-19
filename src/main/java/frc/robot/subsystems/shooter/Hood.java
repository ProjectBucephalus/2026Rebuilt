package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.Conversions;
import static frc.robot.constants.Constants.HoodConstants.*;

public class Hood {
  private final Servo m_Servo;

  public Hood(int id)
  {
    m_Servo = new Servo(id);
  }

  public void setAngle(int angle)
   {m_Servo.setAngle(Conversions.clamp(angle, minAngle, maxAngle));}
}
