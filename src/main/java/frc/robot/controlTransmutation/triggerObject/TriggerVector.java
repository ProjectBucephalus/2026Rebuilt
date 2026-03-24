package frc.robot.controlTransmutation.triggerObject;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controlTransmutation.FieldObject;
import frc.robot.util.Conversions;

import static frc.robot.constants.Constants.ControlConstants.minAngleTolerance;
import static frc.robot.constants.Constants.ControlConstants.maxAngleTolerance;

/** 
 * Trigger based on robot position and control direction <p/>
 * @note Requires extensive testing 
 * @author 5985
 */
public class TriggerVector extends FieldObject
{
  /** Angle of the robot motion for the final approach, degrees */
  protected double approachHeading;
  /** Angle of the robot motion for the final approach */
  protected Rotation2d approachHeadingRotation;
  /** Point where the approach heading intersects the effect radius */
  protected Translation2d frontCheckpoint;
  /** Point opposite where the approach heading intersects the effect radius */
  protected Translation2d backCheckpoint;
  /** Scalar for how far to back off from the target when approaching from the side */
  protected double approachScalar = 0.1;
  /** Input scale for approaching within the buffer based on distance */
  protected double leadInScalar = 1;
  /** By standard implementation, checkPosition is always run first, which calculates this value */
  private double distance;

  /** Trigger output: true while input vector is towards target */
  private boolean onTarget = false;
  private Trigger trigger = new Trigger(activeSupplier).and(() -> onTarget);

  private Rotation2d lastInputAngle = Rotation2d.kZero;


  /**
   * Trigger monitoring if the control input is towards the target within a certain tollerance
   * @param X x-coordinate of the target
   * @param Y y-coordinate of the target
   * @param approachHeading direction the robot should move to approach the target, degrees anticlockwise
   * @param effectRadius distance from target where the trigger will activate
   * @param targetBuffer distance from the target where the robot is too close to activate the trigger (but can stay active)
   */
  public TriggerVector(double X, double Y, double approachHeading, double effectRadius, double targetBuffer)
  {
    centre = new Translation2d(X, Y);
    this.approachHeading = approachHeading;
    radius = effectRadius;
    buffer = targetBuffer;

    approachHeadingRotation = Rotation2d.fromDegrees(approachHeading);

    frontCheckpoint = centre.minus(new Translation2d(buffer, approachHeadingRotation));
    backCheckpoint  = centre.plus(new Translation2d(buffer, approachHeadingRotation));
  }

  /** @return Trigger monitoring if the control input is towards the target within a certain tollerance */
  public Trigger asTrigger()
    {return trigger;}

  @Override
  public Translation2d process(Translation2d controlInput)
  {
    if 
    (
      activeSupplier.getAsBoolean() && 
      !controlInput.equals(Translation2d.kZero) && 
      checkPosition() && 
      checkAngle(controlInput)
    )
    {
      lastInputAngle = controlInput.getAngle();
      onTarget = true;

      // if (distance <= buffer)
      // {
      //   // TODO: set up PID controller here
      //   Rotation2d angleToTarget = centre.minus(robotPos).getAngle();

      //   return new Translation2d(Math.min(distance * leadInScalar, controlInput.getNorm()), angleToTarget);
      // }
      // else
      // {
      //   double tangentOffset = Math.abs(centre.minus(robotPos).rotateBy(approachHeadingRotation.times(-1)).getY());
      //   Translation2d approachPoint = centre.minus(new Translation2d(buffer + (tangentOffset * approachScalar), approachHeadingRotation));
      //   Rotation2d angleToTarget = approachPoint.minus(robotPos).getAngle();

      //   return new Translation2d(controlInput.getNorm(), angleToTarget);
      // }
    }

    onTarget = false;
    lastInputAngle = Rotation2d.kZero;
    return controlInput;
  }

  /**
   * Checks if the input heading is towards the target
   * @param controlInput Current control input
   * @return True if the attractor should activate
   */
  public boolean checkAngle(Translation2d controlInput)
  {
    if 
    (
      onTarget &&
      !lastInputAngle.equals(Rotation2d.kZero) && 
      Conversions.nearRotation(lastInputAngle, controlInput.getAngle(), minAngleTolerance)
    )
    {
      // If the trigger is active and the control input is similar to last cycle, keep the trigger active
      return true;
    }
    
    if (distance <= buffer)
    {
      // If the robot is within the target buffer (very close to target), just compare input angle to approach heading
      return Conversions.nearRotation(approachHeadingRotation, controlInput.getAngle(), maxAngleTolerance);
    }

    // Calculate current angle from robot to target
    Rotation2d angleToTarget = centre.minus(robotPos).getAngle();
    // Angle tolerance is the angular size of the target buffer, such that the input angle must be towards the buffer area
    double angleTolerance = Conversions.clamp(2*Math.atan(buffer/distance), minAngleTolerance, maxAngleTolerance);
    
    // Compare input angle to the current angle to target
    return Conversions.nearRotation(angleToTarget, controlInput.getAngle(), angleTolerance);
  }

  @Override
  public boolean checkPosition()
  {
    distance = getDistance();
    return
    (
      // Checks if the robot is in the "front" half of the effect radius
      // or is within the target buffer, to prevent false negatives from overshooting
      distance <= buffer ||
      (
        distance <= radius &&
        frontCheckpoint.getDistance(robotPos) <= backCheckpoint.getDistance(robotPos)
      )
    );
  }

  @Override
  public double getDistance()
  {
    // Only need to check distance from robot centre to target centre
    return centre.getDistance(robotPos);
  }

}
