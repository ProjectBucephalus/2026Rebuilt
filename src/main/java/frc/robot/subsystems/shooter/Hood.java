package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import frc.robot.constants.Constants.Interpolation;
import static frc.robot.constants.Constants.Shooter.HoodConstants.*;

/**
 * Interface class for a Servo-driven shooter hood to control altitude.
 * @author 5985
 */
public class Hood 
{
  private final Servo m_Servo;

  /**
   * Creates a Servo driven shooter hood, to be managed by {@link Shooter} master-system
   * @param id PWM-ID of hood altitude servo
   */
  public Hood(int id)
    {m_Servo = new Servo(id);}

  /**
   * Calculate the distance from the shooter to the target
   * 
   * @param shooterPose field-relative shooter pose
   * @param targetPoint Translation2d of the target
   * @return the distance from shooter to the target, metres
   */ 
  private double calculateTargetDist(Pose2d shooterPose, Translation2d targetPoint)
    {return targetPoint.minus(shooterPose.getTranslation()).getNorm();}

  /**
   * Checks if the hood is at the current target altitude <p>
   * NOTE: Current system has no position feedback, so this is an estimation only
   * @return True if altitude is within tollerance
   */
  public boolean atAltitude()
  {
    // TODO If large changes are requested, estimate travel time assuming 1.5s to cover full range
    return true;
  }

  /**
   * Intended to be called in {@link Shooter#periodic()} <p>
   * Recalculate the target altitude and apply it to the motor
   * 
   * @param shooterPose the field-relative shooter pose
   * @param target the current {@link Target}
   */
  public void update(Pose2d shooterPose, Target target)
  {
    // Update the azimuth stored in the target based on the target state
    // Ensures that changing to manual mode doesn't cause sudden motion
    target.altitude = switch (target.state) 
    {
      case Manual -> target.altitude;
      case Point -> Interpolation.shooterAltitudeLow.get(calculateTargetDist(shooterPose, target.point));
      case Hub -> Interpolation.shooterAltitudeHub.get(calculateTargetDist(shooterPose, FieldUtils.getAllianceHubCentre()));
    };

    // Set the angle of the servo to the target angle, 
    // accounting for the gear ratio between the servo and the physical hood 
    // and limiting the target to within the hood's range of motion
    m_Servo.set(Conversions.clamp(target.altitude, 0, hoodRange) / (servoRange * hoodRatio)); // TODO verify this calculation
  }
}
