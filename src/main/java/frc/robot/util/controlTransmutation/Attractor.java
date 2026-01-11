package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;

import static frc.robot.constants.FieldConstants.AutoDrive.*;

/** Guides the robot towards a point along a given heading */
public class Attractor extends FieldObject
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

  private Rotation2d lastInputAngle = Rotation2d.kZero;


  /**
   * Pulls the robot into the target point along the given heading if the control input is within a certain tollerance
   * @param X x-coordinate of the target
   * @param Y y-coordinate of the target
   * @param approachHeading direction the robot should move to approach the target, degrees anticlockwise
   * @param effectRadius distance from target where the robot will start being attracted
   * @param targetBuffer distance from the target where the robot must be traveling along the approach heading
   */
  public Attractor(double X, double Y, double approachHeading, double effectRadius, double targetBuffer)
  {
    centre = new Translation2d(X, Y);
    this.approachHeading = approachHeading;
    radius = effectRadius;
    buffer = targetBuffer;

    approachHeadingRotation = Rotation2d.fromDegrees(approachHeading);

    frontCheckpoint = centre.minus(new Translation2d(buffer, approachHeadingRotation));
    backCheckpoint  = centre.plus(new Translation2d(buffer, approachHeadingRotation));
  }

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

      if (distance <= buffer)
      {
        // TODO: set up PID controller here
        Rotation2d angleToTarget = centre.minus(robotPos).getAngle();

        return new Translation2d(Math.min(distance * leadInScalar, controlInput.getNorm()), angleToTarget);
      }
      else
      {
        double tangentOffset = Math.abs(centre.minus(robotPos).rotateBy(approachHeadingRotation.times(-1)).getY());
        Translation2d approachPoint = centre.minus(new Translation2d(buffer + (tangentOffset * approachScalar), approachHeadingRotation));
        Rotation2d angleToTarget = approachPoint.minus(robotPos).getAngle();

        return new Translation2d(controlInput.getNorm(), angleToTarget);
      }
    }

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
      !lastInputAngle.equals(Rotation2d.kZero) && 
      Conversions.isRotationNear(lastInputAngle, controlInput.getAngle(), minAngleTolerance)
    )
    {
      return true;
    }
    
    if (distance <= buffer)
    {
      return Conversions.isRotationNear(approachHeadingRotation, controlInput.getAngle(), maxAngleTolerance);
    }

    Rotation2d angleToTarget = centre.minus(robotPos).getAngle();
    
    double angleTolerance = Conversions.clamp(2*Math.atan(buffer/distance), minAngleTolerance, maxAngleTolerance);

    return Conversions.isRotationNear(angleToTarget, controlInput.getAngle(), angleTolerance);
  }

  @Override
  public boolean checkPosition()
  {
    distance = getDistance();
    return
    (
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
    return centre.getDistance(robotPos);
  }

}
