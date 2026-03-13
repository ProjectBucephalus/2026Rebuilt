package frc.robot.subsystems.vision;

import static frc.robot.constants.Constants.VisionConstants.*;

import java.util.ArrayDeque;
import java.util.Optional;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

/** 
 * Wrapper class to interface with Limelight camera running Photonvision 
 * @author 5985
 */
public class Limelight
{    
  @FunctionalInterface
  public static interface TurretAzimuthSupplier 
  {
    public Pair<Double, Double> get();
  }

  private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private final boolean onTurret;

  private final Transform2d robotToTurret;
  private final Transform2d turretToRobot;
  private final Transform3d structureToCamera;

  private final TimeInterpolatableBuffer<Double> azimuthBuf = TimeInterpolatableBuffer.createDoubleBuffer(azimuthBufLength);
  private final TurretAzimuthSupplier azimuthSup;

  /**
   * Creates a new static Limelight vision camera
   * @param name Device name as published to network
   * @param robotToCamera Transform3d from robot-centre at floor level to the centre of the camera lens
   */
  public Limelight(String name, Transform3d robotToCamera) 
  {
    camera = new PhotonCamera(name);

    robotToTurret = Transform2d.kZero;
    turretToRobot = Transform2d.kZero;
    structureToCamera = robotToCamera;

    photonEstimator = new PhotonPoseEstimator(kTagLayout, structureToCamera);

    azimuthSup = () -> new Pair<>(0.0, 0.0);
    onTurret = false;
  }

 // rotation2d supplier, translation2d assign in constructor + set flag to true (turret to robot)
 /**
  * Creates a new turret-mounted Limelight vision camera
  * @param name Device name as published to network
  * @param turretToCamera Transform3d from turret-centre at floor level to the centre of the camera lens
  * @param turretAngleSup Supplier for the current robot-relative azimuth of the turret, degrees
  * @param robotToTurret Transform2d from robot-centre to turret-centre
  */
  public Limelight(String name, Transform3d turretToCamera, TurretAzimuthSupplier turretAzimuthSup, Transform2d robotToTurret) 
  {
    camera = new PhotonCamera(name);

    this.robotToTurret = robotToTurret;
    turretToRobot = robotToTurret.inverse();
    structureToCamera = turretToCamera;

    photonEstimator = new PhotonPoseEstimator(kTagLayout, structureToCamera);

    this.azimuthSup = turretAzimuthSup;
    onTurret = true;
  }

  /** @param pipelineIndex Vision pipeline index to start using */
  protected void updatePipeline(int pipelineIndex)
    {camera.setPipelineIndex(pipelineIndex);}

  /**
   * Removes uncertain or unwanted tags from the pose estimate before calculating<p>
   * ONLY CALL ONCE PER CYCLE
   * @return Sanitised pose estimate, or an empty Optional if there were no new results
   */
  public Optional<EstimatedRobotPose> getPhotonEst()
  { 
    // Use this call to update some information that should only be done once per cycle
    if (onTurret) updateTurretCache();

    // getAllUnreadResults() should generally only be called once per cycle, as it clears the internal list
    var results = camera.getAllUnreadResults();

    if (results == null || results.isEmpty()) 
      return Optional.empty();

    var result = results.get(results.size() - 1);

    result.targets.removeIf(target -> target.getPoseAmbiguity() > 0.2);

    return photonEstimator.estimateCoprocMultiTagPose(result)
      .or(() -> photonEstimator.estimateLowestAmbiguityPose(result));
  }

  /** Updates the cached turret headings and the current value. ONLY CALL ONCE PER CYCLE */
  private void updateTurretCache()
  {
    var reading = azimuthSup.get();
    azimuthBuf.addSample(reading.getFirst(), reading.getSecond());
  }

  public boolean isOnTurret()
    {return onTurret;}

  public Rotation2d getTurretAngle(double timestamp)
    {return azimuthBuf.getSample(timestamp).map(Rotation2d::fromDegrees).orElse(Rotation2d.kZero);}

    /** @return Transform to convert FROM TURRET to Robot, including current azimuth */
  public Transform2d getRobotToTurret(double timestamp)
  {
    var turretRotation = getTurretAngle(timestamp);
    var translation = robotToTurret.getTranslation().rotateBy(turretRotation);
    var rotation = robotToTurret.getRotation().plus(turretRotation);
    return new Transform2d(translation, rotation);
  }

  /** @return Transform to convert FROM TURRET to Robot, including current azimuth */
  public Transform2d getTurretToRobot(double timestamp)
  {
    var turretRotation = getTurretAngle(timestamp);
    var translation = turretToRobot.getTranslation().rotateBy(turretRotation.unaryMinus());
    var rotation = turretToRobot.getRotation().minus(turretRotation);
    return new Transform2d(translation, rotation);
  }
}
