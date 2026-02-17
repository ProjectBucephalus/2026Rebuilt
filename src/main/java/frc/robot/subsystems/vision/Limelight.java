package frc.robot.subsystems.vision;

import static frc.robot.constants.Constants.VisionConstants.trenchIDs;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

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
  private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private final Transform3d structureToCamera;

  private boolean onTurret = false;
  private DoubleSupplier turretAngleSup;
  private Transform2d robotToTurret;
  
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
    structureToCamera = turretToCamera;
    photonEstimator = new PhotonPoseEstimator(kTagLayout, structureToCamera);
    onTurret = true;
  }

  /** @param pipelineIndex Vision pipeline index to start using */
  protected void updatePipeline(int pipelineIndex)
    {camera.setPipelineIndex(pipelineIndex);}

  /**
   * Removes uncertain or unwanted tags from the pose estimate before calculating
   * @return Sanitised pose estimate, or an empty Optional if there were no new results
   */
  public Optional<EstimatedRobotPose> getPhotonEst()
  { 
    // getAllUnreadResults() should generally only be called once per cycle, as it clears the internal list
    var results = camera.getAllUnreadResults();
    
    if (results == null || results.isEmpty()) 
      return Optional.empty();
    
    var result = results.get(results.size() - 1);

    result.targets.removeIf
      (target -> target.getPoseAmbiguity() > 0.2 || trenchIDs.contains(target.fiducialId));

    return photonEstimator.estimateCoprocMultiTagPose(result)
      .or(() -> photonEstimator.estimateLowestAmbiguityPose(result));
  }

  public boolean isOnTurret()
    {return onTurret;}

  public Rotation2d getTurretAngle()
    {return Rotation2d.fromDegrees(turretAngleSup.getAsDouble());}
  
  /** @return Transform to convert FROM ROBOT to Turret, including current azimuth */
  public Transform2d getRobotToTurret()
    {return new Transform2d(robotToTurret.getTranslation(), robotToTurret.getRotation().minus(getTurretAngle()));}

  /** @return Transform to convert FROM TURRET to Robot, including current azimuth */
  public Transform2d getTurretToRobot()
    {return new Transform2d(robotToTurret.getTranslation().unaryMinus(), robotToTurret.getRotation().plus(getTurretAngle()).unaryMinus());}
}
