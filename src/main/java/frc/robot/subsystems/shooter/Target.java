package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;

/** 
 * A target for the shooter to track, containing the specific state and a set of values to be used across the states 
 * @author 5985
 */
public class Target 
{
  /**
   * The possible targets for the shooter to be tracking
   * 
   * <ul>
   * <li> Manual: Use a fixed, manually-set azimuth and altitude
   * <li> Point: Aim at an arbitrary point
   * <li> Hub: Aim at your alliance's hub
   */
  public enum TargetState 
  {
    /** Use a fixed, manually-set azimuth and altitude */
    Manual, 
    /** Aim at an arbitrary point */
    Point, 
    /** Aim at your alliance's hub */
    Hub;
  }

  /** The currently active {@link TargetState} */
  public TargetState state; 
  /** The target altitude, used for the {@link TargetState#Manual Manual} state, and for storing the last-calculated target in the other two states */
  public double altitude; 
  /** The target azimuth, used for the {@link TargetState#Manual Manual} state, and for storing the last-calculated target in the other two states */
  public double azimuth; 
  /** The target point, used for the {@link TargetState#Point Point} state */
  public Translation2d point;
  /** Offset from target point, used for leading shots while moving and separating ball-streams from multiple shooters */
  public Translation2d offset;
  /** Distance from shooter to target, used for hood angle and flywheel speed */
  public double distance;

  public Target(TargetState state, double altitude, double azimuth, Translation2d point) 
  {
    this.state = state;
    this.altitude = altitude;
    this.azimuth = azimuth;
    this.point = point;
    this.offset = Translation2d.kZero;
  }

  /**
   * Construct a new {@link Target} with {@link Target#altitude altitude}, {@link Target#azimuth azimuth}, and {@link Target#point point} all zeroed
   * @param state the initial state
   */
  public Target(TargetState state) 
    {this(state, 0, 0, Translation2d.kZero);}
}