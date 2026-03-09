package frc.robot.subsystems.vision;

import static frc.robot.constants.Constants.VisionConstants.*;

import java.util.ArrayDeque;
import java.util.Optional;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

/** 
 * Wrapper class to interface with Limelight camera running Photonvision 
 * @author 5985
 */
public class Limelight
{    
  private final PhotonCamera camera;
  private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
  private final Transform3d structureToCamera;
  private final PhotonPoseEstimator photonEstimator;
  private boolean onTurret = false;
  private DoubleSupplier turretAngleSup;
  private Transform2d robotToTurret;
  private Transform2d turretToRobot;
  private Queue<Double> turretCache = new ArrayDeque<>(5);
  private Rotation2d currentTurretAngle;

  /**
   * Creates a new static Limelight vision camera
   * @param name Device name as published to network
   * @param robotToCamera Transform3d from robot-centre at floor level to the centre of the camera lens
   */
  public Limelight(String name, Transform3d robotToCamera) 
  {
    this.camera = new PhotonCamera(name);
    structureToCamera = robotToCamera;
    photonEstimator = new PhotonPoseEstimator(kTagLayout, structureToCamera);
    
    turretAngleSup = () -> 0;
    robotToTurret = Transform2d.kZero;
    turretToRobot = Transform2d.kZero;
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
  public Limelight(String name, Transform3d turretToCamera, DoubleSupplier turretAngleSup, Transform2d robotToTurret) 
  {
    this.camera = new PhotonCamera(name);
    this.turretAngleSup = turretAngleSup;
    this.robotToTurret = robotToTurret;
    turretToRobot = robotToTurret.inverse();
    structureToCamera = turretToCamera;
    photonEstimator = new PhotonPoseEstimator(kTagLayout, structureToCamera);
    onTurret = true;
  }

  /** @param pipelineIndex Vision pipeline index to start using */
  protected void updatePipeline(int pipelineIndex)
    {camera.setPipelineIndex(pipelineIndex);}

  /**
   * Removes uncertain or unwanted tags from the pose estimate before calculating<p>
   * ONLY CALL ONCE PER CYCLE
   * @return Sanitised pose estimate
   */
  public Optional<EstimatedRobotPose> getPhotonEst()
  { 
    // Use this call to update some information that should only be done once per cycle
    double reading = turretAngleSup.getAsDouble();
    turretCache.add(reading);
    if (turretCache.size() > latencyCycles)
      {reading = turretCache.remove();}
    currentTurretAngle = Rotation2d.fromDegrees(reading);

    var results = camera.getAllUnreadResults();

    if (results == null || results.isEmpty()) 
      return Optional.empty();

    var result = results.get(results.size() - 1);

    result.targets.removeIf(target -> target.getPoseAmbiguity() > 0.2);

    return photonEstimator.estimateCoprocMultiTagPose(result)
      .or(() -> photonEstimator.estimateLowestAmbiguityPose(result));
  }

  public boolean isOnTurret()
  {return onTurret;}

  public Rotation2d getTurretAngle()
    {return currentTurretAngle;}
  
  /** @return Transform to convert FROM ROBOT to Turret, including current azimuth */
  public Transform2d getRobotToTurret()
  {return new Transform2d(robotToTurret.getTranslation().rotateBy(getTurretAngle()), robotToTurret.getRotation().plus(getTurretAngle()));}

  /** @return Transform to convert FROM TURRET to Robot, including current azimuth */
  public Transform2d getTurretToRobot()
  {return new Transform2d(turretToRobot.getTranslation().rotateBy(getTurretAngle().unaryMinus()), turretToRobot.getRotation().minus(getTurretAngle()));}
}
