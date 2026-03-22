package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;
import frc.robot.constants.FieldConstants.GeoFencing;
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
  @SuppressWarnings("unused") // May be used in future
  private final AnalogInput io_Altitude;
  
  private final Target target;

  private final boolean inverted;
  private final double homeAngle;

  /**
   * Creates a Servo driven shooter hood, to be managed by {@link Shooter} master-system
   * @param servoID PWM-ID of hood altitude servo
   * @param feedbackID AIO-ID of servo feedback sensor
   * @param inverted Inverts the range and direction of motion of the servo
   * @param targetSup Supplier for current Target object
   */
  public Hood(int servoID, int feedbackID, boolean inverted, double homeAngle, Target target)
  {
    m_Servo = new Servo(servoID);
    io_Altitude = new AnalogInput(feedbackID);
    this.inverted = inverted;
    this.target = target;
    this.homeAngle = homeAngle;
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
  protected void update(Pose2d shooterPose)
  {
    // Limit the target altitude to within the hood's range of motion
    double altitude = Conversions.clamp(target.altitude, 0, hoodRange);
      
    if (target.disabled || GeoFencing.trenchTrigger.getAsBoolean())
      altitude = 0;

    // Convert hood target in degrees to servo position from [0..1]
    double servoTarget = ((altitude * hoodRatio) + homeAngle) / servoRange;

    
    // Invert the target position if needed
    if (inverted) servoTarget = 1 - servoTarget;

    if (target.state == TargetState.Manual || target.disabled)
      // Set the position of the servo to the calculated target position 
      m_Servo.set(servoTarget);
  }
}
