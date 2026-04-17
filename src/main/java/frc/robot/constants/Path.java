package frc.robot.constants;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

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
 * @param nodes List of Pose2d to navigate through, start to end
 */
public record Path(double throttle, Node... nodes)
{
  public record Node(Pose2d pose, double radius) {}

  public Path(double throttle, double pointRadius, Pose2d... poseSequence)
  {
    this(throttle, new Node[poseSequence.length]);

    for (int i = 0; i < poseSequence.length; i++)
      nodes[i] = new Node(poseSequence[i], pointRadius);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o instanceof Path path)
      return throttle == path.throttle && Arrays.equals(nodes, path.nodes);
    else 
      return false;
  }

  @Override
  public int hashCode() 
    {return 31 * Objects.hash(throttle) + Arrays.hashCode(nodes);}

  @Override
  public String toString() {
    return "Path{" +
            "throttle=" + throttle +
            ", nodes=" + Arrays.toString(nodes) +
            '}';
  }

  public Pose2d targetPose()
    {return nodes[nodes.length - 1].pose();}

  /** Creates a clone of the path, rotated around field-centre */
  public Path rotated()
  {
    var rotatedSequence = new Node[nodes.length];
    for (int i = 0; i < nodes.length; i++)
      rotatedSequence[i] = new Node(FieldUtils.rotatePose(nodes[i].pose()), nodes[i].radius());

    return new Path(throttle, rotatedSequence);
  }

  /** Creates a Red Alliance clone of the original Blue Alliance path */
  public Path allianceRotated()
  {
    return switch (FieldUtils.getAlliance()) { case Blue -> this; case Red -> this.rotated(); };
  }

  public Path concat(Path other)
  {
    Node[] concatSequence = Arrays.copyOf(nodes, nodes.length + other.nodes.length);
    System.arraycopy(other.nodes, 0, concatSequence, nodes.length, other.nodes.length);
    return new Path(throttle, concatSequence);
  }

  public void display(String fieldObject)
  {
    var poses = Arrays.stream(nodes).map(Node::pose).toArray(Pose2d[]::new);
    PBDash.addToFieldObject(fieldObject, poses);
  }

  public static final Map<String, Path> autoPaths = new HashMap<>();
  static
  {
    // Right side trench, alliance zone -> mid zone
    autoPaths.put
    (
      "r_trench_a2m", 
      new Path
      (
        0.5,
        0.2, 
        new Pose2d(2.25, 0.8, Rotation2d.kZero),
        new Pose2d(3.25, 0.55, Rotation2d.kZero),
        new Pose2d(5.5, 0.55, Rotation2d.kZero),
        new Pose2d(6.5, 0.8, Rotation2d.kZero)
      )
    );
    // 2: Left side trench, alliance zone -> mid zone
    autoPaths.put
    (
      "l_trench_a2m", 
      new Path
      (
        0.5,
        0.1, 
        new Pose2d(2.5, 7.53, Rotation2d.kZero),
        new Pose2d(6.25, 7.53, Rotation2d.kZero)
      )
    );
    // 3: Right side trench, mid zone -> alliance zone
    autoPaths.put
    (
      "r_trench_m2a", 
      new Path
      (
        0.5,
        0.2, 
        new Pose2d(6.5, 0.8, Rotation2d.k180deg),
        new Pose2d(5.5, 0.55, Rotation2d.k180deg),
        new Pose2d(3.25, 0.55, Rotation2d.k180deg),
        new Pose2d(2.25, 0.8, Rotation2d.k180deg)
      )
    );
    // 4: Left side trench, mid zone -> alliance zone
    autoPaths.put
    (
      "l_trench_m2a", 
      new Path
      (
        0.5,
        0.1, 
        new Pose2d(6.75, 7.53, Rotation2d.k180deg),
        new Pose2d(3, 7.53, Rotation2d.k180deg)
      )
    );
    // 5: Right side mid zone ball collection
    autoPaths.put
    (
      "r_balls", 
      new Path
      (
        1.0,
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
        1.0,
        0.5, 
        new Pose2d(7.75, 7.455, Rotation2d.kCW_90deg),
        new Pose2d(7.75, 3.08, Rotation2d.kCW_90deg)
      )
    );
  }

  public static final Path climbBlueRight = new Path
  (
    0.2,
    0.3, 
    new Pose2d(GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCW_90deg),
    new Pose2d(GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCW_90deg)
  );
  
  public static final Path climbBlueLeft = new Path
  (
    0.2,
    0.3, 
    new Pose2d(GeoFencing.towerPostBlueN.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCCW_90deg),
    new Pose2d(GeoFencing.towerPostBlueN.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCCW_90deg)
  );

  public static final Path climbRedRight = new Path
  (
    0.2,
    0.3, 
    new Pose2d(GeoFencing.towerPostRedN.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCCW_90deg),
    new Pose2d(GeoFencing.towerPostRedN.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCCW_90deg)
  );
  
  public static final Path climbRedLeft = new Path
  (
    0.2,
    0.3, 
    new Pose2d(GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCW_90deg),
    new Pose2d(GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbEndOffset), Rotation2d.kCW_90deg)
  );
}
