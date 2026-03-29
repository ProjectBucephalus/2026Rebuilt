package frc.robot.constants;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.util.FieldUtils;

/**
 * Defines a path to be used by PathFollowDrive commands <p>
 * Note: Assumed to be Blue Alliance for use with {@link Path#allianceRotated() allianceRotated()} method
 * @param pointRadius Approach distance before switching to next point, metres
 * @param heading Rotation for robot to face, applies over entire path
 * @param sequence List of Translation2d to navigate through, start to end
 */
public record Path(double pointRadius, Rotation2d heading, Translation2d... sequence)
{
  public Pose2d targetPose()
    {return new Pose2d(sequence[sequence.length - 1], heading);}

  /** Creates a clone of the path, rotated around field-centre */
  public Path rotated()
  {
    Translation2d[] rotatedSequence = new Translation2d[sequence.length];
    for (int i = 0; i < sequence.length; i++)
      rotatedSequence[i] = FieldUtils.rotateTranslation(sequence[i]);

    return new Path
    (
      pointRadius,
      heading.rotateBy(Rotation2d.k180deg),
      rotatedSequence
    );
  }

  /** Creates a Red Alliance clone of the original Blue Alliance path */
  public Path allianceRotated()
  {
    return switch (FieldUtils.getAlliance()) { case Blue -> this; case Red -> this.rotated(); };
  }

  public static final HashMap<String, Path> autoPaths = new HashMap<>();
  static
  {
    // Right side trench, alliance zone -> mid zone
    autoPaths.put
    (
      "r_trench_a2m", 
      new Path
      (
        0.75, 
        Rotation2d.kZero, 
        new Translation2d(2.5, 0.625),
        new Translation2d(6.25, 0.625)
      )
    );
    // 2: Left side trench, alliance zone -> mid zone
    autoPaths.put
    (
      "l_trench_a2m", 
      new Path
      (
        0.75, 
        Rotation2d.kZero, 
        new Translation2d(2.5, 7.4),
        new Translation2d(6.25, 7.4)
      )
    );
    // 3: Right side trench, mid zone -> alliance zone
    autoPaths.put
    (
      "r_trench_m2a", 
      new Path
      (
        0.5, 
        Rotation2d.k180deg, 
        new Translation2d(6.75, 0.625),
        new Translation2d(3, 0.625)
      )
    );
    // 4: Left side trench, mid zone -> alliance zone
    autoPaths.put
    (
      "l_trench_m2a", 
      new Path
      (
        0.5, 
        Rotation2d.k180deg, 
        new Translation2d(6.75, 7.4),
        new Translation2d(3, 7.4)
      )
    );
    // 5: Right side mid zone ball collection
    autoPaths.put
    (
      "r_balls", 
      new Path
      (
        1, 
        Rotation2d.kCCW_90deg, 
        new Translation2d(7.75, 0.625),
        new Translation2d(7.75, 5)
      )
    );
    // 6: Left side mid zone ball collection
    autoPaths.put
    (
      "l_balls", 
      new Path
      (
        1, 
        Rotation2d.kCW_90deg, 
        new Translation2d(7.75, 0.625),
        new Translation2d(7.75, 5.58)
      )
    );
  };

  public static final Path testPath = new Path
  (
    1,    
    Rotation2d.k180deg,
    new Translation2d(15, 2),
    new Translation2d(11, 2),
    new Translation2d(11, 6)
  );

  public static final Path climbBlueRight = new Path
  (
    0.3, 
    Rotation2d.kCW_90deg, 
    GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbStartOffset),
    GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbEndOffset)
  );
  
  public static final Path climbBlueLeft = new Path
  (
    0.3, 
    Rotation2d.kCCW_90deg, 
    GeoFencing.towerPostBlueN.getCentre().plus(GeoFencing.climbStartOffset),
    GeoFencing.towerPostBlueN.getCentre().plus(GeoFencing.climbEndOffset)
  );

  public static final Path climbRedRight = new Path
  (
    0.3, 
    Rotation2d.kCCW_90deg, 
    GeoFencing.towerPostRedN.getCentre().plus(GeoFencing.climbStartOffset),
    GeoFencing.towerPostRedN.getCentre().plus(GeoFencing.climbEndOffset)
  );
  
  public static final Path climbRedLeft = new Path
  (
    0.3, 
    Rotation2d.kCW_90deg, 
    GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbStartOffset),
    GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbEndOffset)
  );
}
