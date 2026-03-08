package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;

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
  public static enum TargetState 
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
  /** The target azimuth, used for the {@link TargetState#Manual Manual} state, and for storing the last-calculated target in the other two states */
  @Logged(name = "Target Azimuth Degrees")
  public double azimuth = 0; 
  /** The target altitude, used for the {@link TargetState#Manual Manual} state, and for storing the last-calculated target in the other two states */
  @Logged(name = "Target Altitude Degrees")
  public double altitude = 0; 
  /** The target point, used for the {@link TargetState#Point Point} state */
  public Translation2d point = Translation2d.kZero;
  /** Offset from target point, used for leading shots while moving and separating ball-streams from multiple shooters */
  protected Translation2d offset = Translation2d.kZero;
  /** Distance from shooter to target, used for hood angle and flywheel speed */
  @Logged(name = "Target Distance Meters")
  protected double distance = 0;
  @Logged(name = "Target Speed Rotations per Second")
  public double speed = 0;
  /** Flywheels will maintain speed while {@code true}, will idle when {@code false} */
  public boolean flywheelsActive = false;

  /**
   * Construct a new {@link Target} with all values zeroed
   * @param state the initial state
   */
  public Target(TargetState state) 
    {this.state = state;}
}