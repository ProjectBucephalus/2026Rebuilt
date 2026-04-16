package frc.robot.constants;

import java.util.Arrays;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;

/**
 * Defines a path to be used by PathFollowDrive commands <p>
 * Note: Assumed to be Blue Alliance for use with {@link Path#allianceRotated() allianceRotated()} method
 * @param pointRadius Approach distance before switching to next point, metres
 * @param heading Rotation for robot to face, applies over entire path
 * @param sequence List of Pose2d to navigate through, start to end
 */
public record Path(Node... sequence)
{
  public record Node(Pose2d pose, double radius) {}

  public Path(double pointRadius, Pose2d... poseSequence)
  {
    this(new Node[poseSequence.length]);

    for (int i = 0; i < poseSequence.length; i++)
      sequence[i] = new Node(poseSequence[i], pointRadius);
  }

  public Pose2d targetPose()
    {return sequence[sequence.length - 1].pose();}

  /** Creates a clone of the path, rotated around field-centre */
  public Path rotated()
  {
    var rotatedSequence = new Node[sequence.length];
    for (int i = 0; i < sequence.length; i++)
      rotatedSequence[i] = new Node(FieldUtils.rotatePose(sequence[i].pose()), sequence[i].radius());

    return new Path(rotatedSequence);
  }

  /** Creates a Red Alliance clone of the original Blue Alliance path */
  public Path allianceRotated()
  {
    return switch (FieldUtils.getAlliance()) { case Blue -> this; case Red -> this.rotated(); };
  }

  public Path concat(Path other)
  {
    Node[] concatSequence = Arrays.copyOf(sequence, sequence.length + other.sequence.length);
    System.arraycopy(other.sequence, 0, concatSequence, sequence.length, other.sequence.length);
    return new Path(concatSequence);
  }

  public void display(String fieldObject)
  {
    var poses = Arrays.stream(sequence).map(node -> node.pose()).toArray(Pose2d[]::new);
    PBDash.addToFieldObject(fieldObject, poses);
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
        0.25, 
        new Pose2d(2.5, 0.625, Rotation2d.kZero),
        new Pose2d(6.25, 0.625, Rotation2d.kZero)
      )
    );
    // 2: Left side trench, alliance zone -> mid zone
    autoPaths.put
    (
      "l_trench_a2m", 
      new Path
      (
        0.25, 
        new Pose2d(2.5, 7.4, Rotation2d.kZero),
        new Pose2d(6.25, 7.4, Rotation2d.kZero)
      )
    );
    // 3: Right side trench, mid zone -> alliance zone
    autoPaths.put
    (
      "r_trench_m2a", 
      new Path
      (
        0.25, 
        new Pose2d(6.75, 0.625, Rotation2d.k180deg),
        new Pose2d(3, 0.625, Rotation2d.k180deg)
      )
    );
    // 4: Left side trench, mid zone -> alliance zone
    autoPaths.put
    (
      "l_trench_m2a", 
      new Path
      (
        0.25, 
        new Pose2d(6.75, 7.4, Rotation2d.k180deg),
        new Pose2d(3, 7.4, Rotation2d.k180deg)
      )
    );
    // 5: Right side mid zone ball collection
    autoPaths.put
    (
      "r_balls", 
      new Path
      (
        0.5, 
        new Pose2d(7.75, 0.625, Rotation2d.kCCW_90deg),
        new Pose2d(7.75, 5, Rotation2d.kCCW_90deg)
      )
    );
    // 6: Left side mid zone ball collection
    autoPaths.put
    (
      "l_balls", 
      new Path
      (
        0.5, 
        new Pose2d(7.75, 7.455, Rotation2d.kCW_90deg),
        new Pose2d(7.75, 3.08, Rotation2d.kCW_90deg)
      )
    );
  };

  public static final Path climbBlueRight = new Path
  (
    0.3, 
    new Pose2d(GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCW_90deg),
    new Pose2d(GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCW_90deg)
  );
  
  public static final Path climbBlueLeft = new Path
  (
    0.3, 
    new Pose2d(GeoFencing.towerPostBlueN.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCCW_90deg),
    new Pose2d(GeoFencing.towerPostBlueN.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCCW_90deg)
  );

  public static final Path climbRedRight = new Path
  (
    0.3, 
    new Pose2d(GeoFencing.towerPostRedN.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCCW_90deg),
    new Pose2d(GeoFencing.towerPostRedN.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCCW_90deg)
  );
  
  public static final Path climbRedLeft = new Path
  (
    0.3, 
    new Pose2d(GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCW_90deg),
    new Pose2d(GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCW_90deg)
  );
}
