package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

import static frc.robot.constants.Constants.VisionConstants.*;
import static frc.robot.constants.FieldConstants.tagLayout;

/** 
 * Wrapper class to interface with Limelight camera running Photonvision 
 * @author 5985
 */
public class Limelight
{    
  private final String name;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private final boolean onTurret;
  private boolean active = true;

  private final Transform2d robotToTurret;
  private final Transform2d turretToRobot;
  private final Transform2d cameraToStructure;

  private final TimeInterpolatableBuffer<Double> azimuthBuf = TimeInterpolatableBuffer.createDoubleBuffer(azimuthBufLength);
  private final Supplier<Pair<Double, Double>> azimuthSup;

  /**
   * Creates a new static Limelight vision camera
   * @param name Device name as published to network
   * @param cameraToRobot Transform2d from the centre of the camera body to robot-centre
   */
  public Limelight(String name, Transform2d cameraToRobot) 
  {
    this.name = name;
    camera = new PhotonCamera(name);

    robotToTurret = Transform2d.kZero;
    turretToRobot = Transform2d.kZero;
    cameraToStructure = cameraToRobot;

    photonEstimator = new PhotonPoseEstimator(tagLayout, Transform3d.kZero);

    azimuthSup = () -> new Pair<>(0.0, 0.0);
    onTurret = false;
  }

 // rotation2d supplier, translation2d assign in constructor + set flag to true (turret to robot)
 /**
  * Creates a new turret-mounted Limelight vision camera
  * @param name Device name as published to network
  * @param cameraToTurret Transform2d from the centre of the camera body to turret-centre
  * @param turretAngleSup Supplier for the current robot-relative azimuth of the turret, degrees
  * @param robotToTurret Transform2d from robot-centre to turret-centre
  */
  public Limelight(String name, Transform2d cameraToTurret, Supplier<Pair<Double, Double>> turretAzimuthSup, Transform2d robotToTurret) 
  {
    this.name = name;
    camera = new PhotonCamera(name);

    this.robotToTurret = robotToTurret;
    turretToRobot = robotToTurret.inverse();
    cameraToStructure = cameraToTurret;

    photonEstimator = new PhotonPoseEstimator(tagLayout, Transform3d.kZero);

    this.azimuthSup = turretAzimuthSup;
    onTurret = true;
  }

  public String getName()
    {return name;}

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
    if (results == null || results.isEmpty()) return Optional.empty();

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

  /** @return Transform to convert from Camera to Structure */
  public Transform2d getCameraToStructure()
    {return cameraToStructure;}

  public boolean isActive()
  {return active;}

  public void setActive(boolean activeState)
  {active = activeState;}
}
