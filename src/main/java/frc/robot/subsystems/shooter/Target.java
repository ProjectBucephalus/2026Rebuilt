package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.ShooterConstants;

/** 
 * A target for the shooter to track, containing the specific state and a set of values to be used across the states 
 * @author 5985
 */
@Logged
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
    Hub,
    /** Fixed position to give camera optimal view */
    Vision;
  }

  /** The current translation of the shooter in field space */
  public Translation2d shooterPosition = Translation2d.kZero;
  /** The currently active {@link TargetState} */
  public TargetState state; 
  /** The target azimuth, used for the {@link TargetState#Manual Manual} state, and for storing the last-calculated target in the other two states */
  @Logged(name = "azimuth Degrees")
  public double azimuth = 0; 
  /** The target altitude, used for the {@link TargetState#Manual Manual} state, and for storing the last-calculated target in the other two states */
  @Logged(name = "altitude Degrees")
  public double altitude = 0; 
  /** The target point, used for the {@link TargetState#Point Point} state */
  public Translation2d point = Translation2d.kZero;
  /** Offset from target point, used for leading shots while moving and separating ball-streams from multiple shooters */
  protected Translation2d offset = Translation2d.kZero;
  /** Distance from shooter to target, used for hood angle and flywheel speed */
  @Logged(name = "distance Meters")
  public double distance = ShooterConstants.minRange;
  @Logged(name = "speed Rotations per Second")
  public double speed = 0;
  /** Flywheels will maintain speed while {@code true}, will idle when {@code false} */
  public boolean flywheelsActive = false;
  public boolean disabled = false;

  /**
   * Construct a new {@link Target} with all values zeroed
   * @param state the initial state
   */
  public Target(TargetState state) 
    {this.state = state;}
}