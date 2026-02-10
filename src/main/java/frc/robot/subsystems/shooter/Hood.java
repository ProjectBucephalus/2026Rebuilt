package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import frc.robot.constants.Constants.Interpolation;
import static frc.robot.constants.Constants.ShooterConstants.HoodConstants.*;

import java.util.function.Supplier;

/**
 * Interface class for a Servo-driven shooter hood to control altitude.
 * @author 5985
 */
public class Hood 
{
  private final Servo m_Servo;
  private final AnalogInput io_Altitude;
  private final boolean inverted;
  private final Supplier<Target> targetSup;


  /**
   * Creates a Servo driven shooter hood, to be managed by {@link Shooter} master-system
   * @param servoID PWM-ID of hood altitude servo
   * @param feedbackID AIO-ID of servo feedback sensor
   * @param inverted Inverts the range and direction of motion of the servo
   * @param targetSup Supplier for current Target object
   */
  public Hood(int servoID, int feedbackID, boolean inverted, Supplier<Target> targetSup)
  {
    m_Servo = new Servo(servoID);
    io_Altitude = new AnalogInput(feedbackID);
    this.inverted = inverted;
    this.targetSup = targetSup;
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
  public void update(Pose2d shooterPose)
  {
    var target = targetSup.get();
    // Update the azimuth stored in the target based on the target state
    // Ensures that changing to manual mode doesn't cause sudden motion
    target.altitude = switch (target.state) 
    {
      case Manual -> target.altitude;
      case Point -> Interpolation.shooterAltitudeLow.get(target.distance);
      case Hub -> Interpolation.shooterAltitudeHub.get(target.distance);
    };

    // Limit the target altitude to within the hood's range of motion
    target.altitude = Conversions.clamp(target.altitude, 0, hoodRange);

    // Convert hood target in degrees to servo position from [0..1]
    double servoTarget = (target.altitude * hoodRatio) / servoRange;

    // Invert the target position if needed
    if (inverted) servoTarget = 1 - servoTarget;

    // Set the position of the servo to the calculated target position 
    m_Servo.set(servoTarget);
  }
}
