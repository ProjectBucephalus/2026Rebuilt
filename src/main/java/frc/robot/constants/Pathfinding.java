package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.FieldUtils;

/** 
 * Constraints and waypoint sequences for use with PathFollowDrive commands
 * @author 5985
 */
public abstract class Pathfinding 
{
  /**
   * Defines a path to be used by PathFollowDrive commands <p>
   * Note: Assumed to be Blue Alliance for use with {@code allianceRotated()} function
   * @param pointRadius Approach distance before switching to next point, metres
   * @param heading Rotation for robot to face, applies over entire path
   * @param sequence List of Translation2d to navigate through, start to end
   */
  public record Path(double pointRadius, Rotation2d heading, Translation2d... sequence)
  {
    /** Creates a clone of the path, rotated around field-centre */
    public Path rotated()
    {
      Translation2d[] rotatedSequence = sequence.clone();
      for (var t : rotatedSequence)
        t = FieldUtils.rotateTranslation(t);

      return new Path
      (
        pointRadius,
        heading.unaryMinus(),
        rotatedSequence
      );
    }

    /** Creates a Red Alliance clone of the original Blue Alliance path */
    public Path allianceRotated()
    {
      if (FieldUtils.isRedAlliance())
        return this.rotated();
      else
        return this;
    }
  }

  public static final Path testPath = new Path
  (
    1,    
    Rotation2d.k180deg,
    new Translation2d(15, 2),
    new Translation2d(11, 2),
    new Translation2d(11, 6)
  );
}
