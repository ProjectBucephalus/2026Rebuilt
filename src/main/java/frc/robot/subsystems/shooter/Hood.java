package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.Conversions;
import frc.robot.constants.FieldConstants.GeoFencing;

import static frc.robot.constants.Constants.ShooterConstants.HoodConstants.*;

/**
 * Interface class for a Servo-driven shooter hood to control altitude.
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Hood 
{
  private final Servo m_Servo;

  private final boolean inverted;
  private final double homeAngle;

  private final Target target;

  /**
   * Creates a Servo driven shooter hood, to be managed by {@link Shooter} master-system
   * @param servoID PWM-ID of hood altitude servo
   * @param inverted Inverts the range and direction of motion of the servo
   * @param homeAngle Zero-point offset for the servo
   * @param target Target object for the shooter
   */
  public Hood(int servoID, boolean inverted, double homeAngle, Target target)
  {
    m_Servo = new Servo(servoID);
    this.inverted = inverted;
    this.homeAngle = homeAngle;
    this.target = target;
  }

  /**
   * Intended to be called in {@link Shooter#periodic()} <p>
   * Converts the target altitude to motor position and applies that to the motor
   */
  protected void update()
  {
    // Limit the target altitude to within the hood's range of motion     
    double altitude = (target.disabled || GeoFencing.trenchTrigger.getAsBoolean())
      ? 0 
      : Conversions.clamp(target.altitude, 0, hoodRange);

    // Convert hood target in degrees to servo position from [0..1]
    double servoTarget = ((altitude * hoodRatio) + homeAngle) / servoRange;

    // Set the position of the servo to the calculated target position, inverting if needed
    m_Servo.set(inverted ? 1 - servoTarget : servoTarget);
  }
}
