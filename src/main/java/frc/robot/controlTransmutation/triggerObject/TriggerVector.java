package frc.robot.controlTransmutation.triggerObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controlTransmutation.FieldObject;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;

import static frc.robot.constants.Constants.ControlConstants.minAngleTolerance;

import java.util.function.Supplier;

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
  /** By standard implementation, checkPosition is always run first, which calculates this value */
  private double distance;

  /** Supplier for the controller input to monitor, to allow for use separate from general object list */
  private Supplier<Translation2d> controlInputSup = () -> Translation2d.kZero;
  /** Flag to monitor the given input whenever the trigger value is checked */
  private boolean offlineMonitor = false;

  /** Trigger output: true while input vector is towards target */
  private boolean onTarget = false;
  private Pose2d centrePose;

  /** If false, the trigger cannot *become* true when within the buffer */
  private boolean bufferActivation = true;

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

    centrePose = new Pose2d(centre, approachHeadingRotation);
  }

  /** @return Trigger monitoring if the control input is towards the target within a certain tollerance */
  public Trigger asTrigger()
    {return new Trigger(activeSupplier).and(globalActiveSupplier).and(this::checkTrigger);}

  
  private boolean checkTrigger()
  {
    if (offlineMonitor)
    {
      process(FieldUtils.isAlliance(Alliance.Red) ? controlInputSup.get().unaryMinus() : controlInputSup.get());
    }
    return onTarget;
  }

  /**
   * Sets the given control input to be monitored offline, allowing the trigger to function without explicit processing calls
   * @param controlInputSup Unrotated joystick dual-axis supplier to be checked whenever the trigger is called
   * @return The modified TriggerVector object
   */
  public TriggerVector withControlInput(Supplier<Translation2d> controlInputSup)
  {
    this.controlInputSup = controlInputSup;
    offlineMonitor = true;
    return this;
  }

  @Override
  public Translation2d process(Translation2d controlInput)
  {
    if 
    (
      globalActiveSupplier.getAsBoolean() &&
      activeSupplier.getAsBoolean() && 
      !controlInput.equals(Translation2d.kZero) && 
      checkPosition() && 
      checkAngle(controlInput)
    )
    {
      lastInputAngle = controlInput.getAngle();
      onTarget = true;
      PBDash.putFieldObject("Active Trigger Vector", centrePose);
    }
    else
    {
      if (onTarget)
        PBDash.putFieldObject("Active Trigger Vector");
      onTarget = false;
      lastInputAngle = Rotation2d.kZero;
    }

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
      // Has the option to not activate when close
      return bufferActivation && Conversions.nearRotation(approachHeadingRotation, controlInput.getAngle(), minAngleTolerance);
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
    // Only need to check distance from robot to target centre
    return centre.getDistance(robotPos) - robotRadius;
  }

  /**
   * Sets whether the trigger should activate when the robot is within the buffer, or only activate on approach
   * @param bufferActivation {@code false} to prevent activation when within the buffer. Default {@code true}
   * @return The modified TriggerVector object
   */
  public TriggerVector withBufferActivation(boolean bufferActivation)
  {
    this.bufferActivation = bufferActivation;
    return this;
  }
}
