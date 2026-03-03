package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.Conversions;

import frc.robot.constants.Constants.Interpolation;
import frc.robot.subsystems.shooter.Target.TargetState;

import static frc.robot.constants.Constants.ShooterConstants.HoodConstants.*;

/**
 * Interface class for a Servo-driven shooter hood to control altitude.
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Hood 
{
  private final Servo m_Servo;
  private final AnalogInput io_Altitude;
  private final boolean inverted;
  private final Target target;

  @Logged(name = "Target Altitude")
  private double altitude;

  /**
   * Creates a Servo driven shooter hood, to be managed by {@link Shooter} master-system
   * @param servoID PWM-ID of hood altitude servo
   * @param feedbackID AIO-ID of servo feedback sensor
   * @param inverted Inverts the range and direction of motion of the servo
   * @param targetSup Supplier for current Target object
   */
  public Hood(int servoID, int feedbackID, boolean inverted, Target target)
  {
    m_Servo = new Servo(servoID);
    io_Altitude = new AnalogInput(feedbackID);
    this.inverted = inverted;
    this.target = target;
  }

  /**
   * Checks if the hood is at the current target altitude <p>
   * NOTE: Current system has no position feedback, so this is an estimation only
   * @return True if altitude is within tollerance
   */
  public boolean atAltitude()
  {
    // TODO Use analog feedback from servo, also return true if raw output is 0
    return true;
  }

  /**
   * Intended to be called in {@link Shooter#periodic()} <p>
   * Recalculate the target altitude and apply it to the motor
   * 
   * @param shooterPose the field-relative shooter pose
   */
  protected void update()
  {
    var interpTable = 
      target.state == TargetState.Point ? 
      Interpolation.shooterAltitudeLow : 
      Interpolation.shooterAltitudeHub;

    // Limit the target altitude to within the hood's range of motion
    altitude = Conversions.clamp(interpTable.get(target.distance), 0, hoodRange);

    // Convert hood target in degrees to servo position from [0..1]
    double servoTarget = ((altitude + homeAngle) * hoodRatio) / servoRange;

    // Invert the target position if needed
    if (inverted) servoTarget = 1 - servoTarget;

    // Set the position of the servo to the calculated target position 
    m_Servo.set(servoTarget);
  }
}
