package frc.robot.constants;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants.FieldTuning;
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

  /**
   * Creates a clone of the path with all waypoints offset
   * @param x Offset on the x-axis (Out from Blue), metres
   * @param y Offset on the y-axis (Left from Blue), metres
   * @return The modified path
   */
  public Path offset(double x, double y)
  {
    Translation2d offsetXY = new Translation2d(x, y);
    var offsetSequence = new Node[nodes.length];
    
    for (int i = 0; i < nodes.length; i++)
      offsetSequence[i] = new Node
      (
        new Pose2d(nodes[i].pose.getTranslation().plus(offsetXY), nodes[i].pose.getRotation()), 
        nodes[i].radius()
      );

    return new Path(throttle, offsetSequence);
  }

  /**
   * Creates a clone of the path with all waypoints offset relative to drivers
   * @param out Offset Out, metres
   * @param left Offset Left, metres
   * @return The modified path
   */
  public Path allianceOffset(double out, double left)
    {return FieldUtils.isAlliance(Alliance.Blue) ? offset(out, left) : offset(-out, -left);}

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
    // 1: Right side trench, alliance zone -> mid zone
    autoPaths.put
    (
      "r_trench_a2m", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(2.25, 0.8, Rotation2d.kZero),
        new Pose2d(3.25, 0.61, Rotation2d.kZero),
        new Pose2d(5.5, 0.61, Rotation2d.kZero),
        new Pose2d(6.5, 0.8, Rotation2d.kZero)
      )
    );
    // 2: Left side trench, alliance zone -> mid zone
    autoPaths.put
    (
      "l_trench_a2m", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(2.25, 7.22, Rotation2d.kZero),
        new Pose2d(3.25, 7.47, Rotation2d.kZero),
        new Pose2d(5.5, 7.47, Rotation2d.kZero),
        new Pose2d(6.5, 7.22, Rotation2d.kZero)
      )
    );
    // 3: Right side trench, mid zone -> alliance zone
    autoPaths.put
    (
      "r_trench_m2a", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(6.5, 0.8, Rotation2d.k180deg),
        new Pose2d(5.5, 0.61, Rotation2d.k180deg),
        new Pose2d(3.25, 0.61, Rotation2d.k180deg),
        new Pose2d(2.25, 0.8, Rotation2d.k180deg)
      )
    );
    // 4: Left side trench, mid zone -> alliance zone
    autoPaths.put
    (
      "l_trench_m2a", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(6.5, 7.22, Rotation2d.k180deg),
        new Pose2d(5.5, 7.47, Rotation2d.k180deg),
        new Pose2d(3.25, 7.47, Rotation2d.k180deg),
        new Pose2d(2.25, 7.22, Rotation2d.k180deg)
      )
    );
    // 5: Right side mid zone ball collection
    autoPaths.put
    (
      "r_balls", 
      new Path
      (
        0.7,
        0.5, 
        new Pose2d(7.25, 1, Rotation2d.kCCW_90deg),
        new Pose2d(7.75, 1.5, Rotation2d.kCCW_90deg),
        new Pose2d(7.75, 3.5, Rotation2d.kCCW_90deg)
      )
    );
    // 6: Left side mid zone ball collection
    autoPaths.put
    (
      "l_balls", 
      new Path
      (
        0.7,
        0.5, 
        new Pose2d(7.25, 7.08, Rotation2d.kCW_90deg),
        new Pose2d(7.75, 6.58, Rotation2d.kCW_90deg),
        new Pose2d(7.75, 4.58, Rotation2d.kCW_90deg)
      )
    );

    // 7: Right side trench, alliance zone -> mid zone, facing alliance zone
    autoPaths.put
    (
      "r_trench_a2m_r", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(2.25, 0.8, Rotation2d.k180deg),
        new Pose2d(3.25, 0.61, Rotation2d.k180deg),
        new Pose2d(5.5, 0.61, Rotation2d.k180deg),
        new Pose2d(6.5, 0.8, Rotation2d.k180deg)
      )
    );
    // 8: Left side trench, alliance zone -> mid zone, facing alliance zone
    autoPaths.put
    (
      "l_trench_a2m_r", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(2.25, 7.22, Rotation2d.k180deg),
        new Pose2d(3.25, 7.47, Rotation2d.k180deg),
        new Pose2d(5.5, 7.47, Rotation2d.k180deg),
        new Pose2d(6.5, 7.22, Rotation2d.k180deg)
      )
    );
    // 9: Right side mid zone ball collection, near alliance zone
    autoPaths.put
    (
      "r_balls_near", 
      new Path
      (
        0.7,
        0.5, 
        new Pose2d(6.75, 1, Rotation2d.kCCW_90deg),
        new Pose2d(6.75, 2.5, Rotation2d.kCCW_90deg),
        new Pose2d(6.25, 3.5, Rotation2d.kCCW_90deg)
      )
    );
    // 10: Left side mid zone ball collection, near alliance zone
    autoPaths.put
    (
      "l_balls_near", 
      new Path
      (
        0.7,
        0.5,
        new Pose2d(6.75, 7.08, Rotation2d.kCW_90deg),
        new Pose2d(6.75, 5.58, Rotation2d.kCW_90deg),
        new Pose2d(6.25, 4.58, Rotation2d.kCW_90deg)
      )
    );
    // 11: Depo left to right
    autoPaths.put
    (
      "l_depo", 
      new Path
      (
        0.5,
        0.4, 
        new Pose2d(2.0, 7.25, Rotation2d.k180deg),
        new Pose2d(1.1, 7.0, Rotation2d.k180deg),
        new Pose2d(1.1, 5.5, Rotation2d.k180deg),
        new Pose2d(1.5, 5.25, Rotation2d.fromDegrees(225)),
        new Pose2d(2.0, 5.25, Rotation2d.fromDegrees(225))
      )
    );
    // 12: Depo right to left
    autoPaths.put
    (
      "r_depo", 
      new Path
      (
        0.5,
        0.4, 
        new Pose2d(2.0, 5.25, Rotation2d.k180deg),
        new Pose2d(1.1, 5.5, Rotation2d.k180deg),
        new Pose2d(1.1, 7.0, Rotation2d.k180deg),
        new Pose2d(1.5, 7.25, Rotation2d.fromDegrees(135)),
        new Pose2d(2.0, 7.25, Rotation2d.fromDegrees(135))
      )
    );
    // 13: Right side trench, intake in trench -> mid zone
    autoPaths.put
    (
      "r_trench_i2m", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(4.0, 0.61, Rotation2d.kZero),
        new Pose2d(5.5, 0.61, Rotation2d.kZero),
        new Pose2d(6.5, 0.8, Rotation2d.kZero)
      )
    );
    // 14: Left side trench, intake in trench -> mid zone
    autoPaths.put
    (
      "l_trench_i2m", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(4.0, 7.47, Rotation2d.kZero),
        new Pose2d(5.5, 7.47, Rotation2d.kZero),
        new Pose2d(6.5, 7.22, Rotation2d.kZero)
      )
    );
    // 15: Right side trench, mid zone -> intake in trench
    autoPaths.put
    (
      "r_trench_m2i", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(6.5, 0.8, Rotation2d.kZero),
        new Pose2d(5.5, 0.61, Rotation2d.kZero),
        new Pose2d(4.0, 0.61, Rotation2d.kZero)
      )
    );
    // 16: Left side trench, mid zone -> intake in trench
    autoPaths.put
    (
      "l_trench_m2i", 
      new Path
      (
        0.6,
        0.4, 
        new Pose2d(6.5, 7.22, Rotation2d.kZero),
        new Pose2d(5.5, 7.47, Rotation2d.kZero),
        new Pose2d(4.0, 7.47, Rotation2d.kZero)
      )
    );
  }

  private static final double climbApproachThrottle = 0.3;
  private static final double climbApproachRadius   = 0.3;
  private static final double climbThrottle         = 0.16;
  private static final double climbRadius           = 0.3;

  public static final Path climbApproachBlueRight = new Path
  (
    climbApproachThrottle,
    climbApproachRadius, 
    new Pose2d(GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbApproachOffset), Rotation2d.kCW_90deg),
    new Pose2d(GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCW_90deg)
  );
  
  public static final Path climbApproachBlueLeft = new Path
  (
    climbApproachThrottle,
    climbApproachRadius, 
    new Pose2d(GeoFencing.towerPostBlueN.getCentre().plus(GeoFencing.climbApproachOffset), Rotation2d.kCCW_90deg),
    new Pose2d(GeoFencing.towerPostBlueN.getCentre().plus(GeoFencing.climbStartOffset), Rotation2d.kCCW_90deg)
  );

  public static final Path climbApproachRedRight = new Path
  (
    climbApproachThrottle,
    climbApproachRadius, 
    new Pose2d(GeoFencing.towerPostRedN.getCentre().plus(GeoFencing.climbApproachOffset), Rotation2d.kCCW_90deg),
    new Pose2d(GeoFencing.towerPostRedN.getCentre().plus(GeoFencing.climbStartOffset), Rotation2d.kCCW_90deg)
  );
  
  public static final Path climbApproachRedLeft = new Path
  (
    climbApproachThrottle,
    climbApproachRadius, 
    new Pose2d(GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbApproachOffset), Rotation2d.kCW_90deg),
    new Pose2d(GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbStartOffset), Rotation2d.kCW_90deg)
  );

  public static final Path climbBlueRight = new Path
  (
    climbThrottle,
    climbRadius, 
    new Pose2d(GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbStartOffset), 
               Rotation2d.kCW_90deg.plus(Rotation2d.fromDegrees(FieldTuning.climbAngleBlueRight))),
    new Pose2d(GeoFencing.towerPostBlueS.getCentre().minus(GeoFencing.climbEndOffset), 
               Rotation2d.kCW_90deg.plus(Rotation2d.fromDegrees(FieldTuning.climbAngleBlueRight)))
  );
  
  public static final Path climbBlueLeft = new Path
  (
    climbThrottle,
    climbRadius, 
    new Pose2d(GeoFencing.towerPostBlueN.getCentre().plus(GeoFencing.climbStartOffset), 
               Rotation2d.kCCW_90deg.plus(Rotation2d.fromDegrees(FieldTuning.climbAngleBlueLeft))),
    new Pose2d(GeoFencing.towerPostBlueN.getCentre().plus(GeoFencing.climbEndOffset), 
               Rotation2d.kCCW_90deg.plus(Rotation2d.fromDegrees(FieldTuning.climbAngleBlueLeft)))
  );

  public static final Path climbRedRight = new Path
  (
    climbThrottle,
    climbRadius, 
    new Pose2d(GeoFencing.towerPostRedN.getCentre().plus(GeoFencing.climbStartOffset), 
               Rotation2d.kCCW_90deg.plus(Rotation2d.fromDegrees(FieldTuning.climbAngleRedRight))),
    new Pose2d(GeoFencing.towerPostRedN.getCentre().plus(GeoFencing.climbEndOffset), 
               Rotation2d.kCCW_90deg.plus(Rotation2d.fromDegrees(FieldTuning.climbAngleRedRight)))
  );
  
  public static final Path climbRedLeft = new Path
  (
    climbThrottle,
    climbRadius, 
    new Pose2d(GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbStartOffset), 
               Rotation2d.kCW_90deg.plus(Rotation2d.fromDegrees(FieldTuning.climbAngleRedLeft))),
    new Pose2d(GeoFencing.towerPostRedS.getCentre().minus(GeoFencing.climbEndOffset), 
               Rotation2d.kCW_90deg.plus(Rotation2d.fromDegrees(FieldTuning.climbAngleRedLeft)))
  );
}
