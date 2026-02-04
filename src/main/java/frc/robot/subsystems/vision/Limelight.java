// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.constants.Constants.VisionConstants.trenchIDs;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.PBDash;

/** Wrapper class to interface with Limelight camera running Photonvision */
public class Limelight
{    
  private final PhotonCamera camera;
  // TODO should probably initialise photonEstimator in the constructor so that the tag layout and robotToCam transform can be just restricted to the constructor
  // Also, maybe let the robotToCam transform be provided in the constructor
  private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
  private final Transform3d structureToCamera;
  private final PhotonPoseEstimator photonEstimator;
  private PhotonPipelineResult result;
  private boolean onTurret = false;
  private Supplier<Rotation2d> turretAngleSup;
  private Translation2d turretToRobot;
  

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
      onTurret = false;
    }

 // rotation2d supplier, translation2d assign in constructor + set flag to true (turret to robot)
 /**
  * Creates a new turret-mounted Limelight vision camera
  * @param name Device name as published to network
  * @param turretToCamera Transform3d from turret-centre at floor level to the centre of the camera lens
  * @param turretAngleSup Supplier for the current robot-relative azimuth of the turret, degrees
  * @param turretToRobot Transform2d from robot-centre to turret-centre
  */
  public Limelight(String name, Transform3d turretToCamera, Supplier<Rotation2d> turretAngleSup, Translation2d turretToRobot) 
  {
    this.camera = new PhotonCamera(name);
    this.turretAngleSup = turretAngleSup;
    this.turretToRobot = turretToRobot;
    structureToCamera = turretToCamera;
    photonEstimator = new PhotonPoseEstimator(kTagLayout, structureToCamera);
    onTurret = true;
  }

  /** 
   * Pulls all unread results from the camera and stores the latest <p>
   * Subsequent requests before the camera produces a new result will not clear the stored result
   */
  public void getLatestResult() 
  {
    // getAllUnreadResults() should generally only be called once per cycle, as it clears the internal list
    // The logic here protects against that feature to allow the latest result to be called as needed
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) 
      {result = results.get(results.size()-1);}
  }

  /** @param pipelineIndex Vision pipeline index to start using */
  protected void updatePipeline(int pipelineIndex)
    {camera.setPipelineIndex(pipelineIndex);}

  /**
   * Removes uncertain or unwanted tags from the pose estimate before calculating
   * @return Sanitised pose estimate
   */
  public Optional<EstimatedRobotPose> getPhotonEst()
  { 
    if (result == null) return Optional.empty();

    for (int i = result.targets.size() - 1; i >= 0; i--)
    {
      double targetAmb = result.targets.get(i).getPoseAmbiguity();
      int targetID = result.targets.get(i).fiducialId;
      
      if (targetAmb > 0.2 || trenchIDs.contains(targetID)) 
      {
        result.targets.remove(i);
      } 
    }

    var visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) 
    {
      visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
    }

    return visionEst;
  }

  public boolean isOnTurret()
  {return onTurret;}

  public Rotation2d getTurretAngle()
  {return turretAngleSup.get();}
  
  public Translation2d getTurretToRobot()
  {return turretToRobot;}

  /** 
   * Intended to be called in {@link Vision#periodic()} <p>
   * Pull the latest results from the camera ready to be used
   */
  public void update() 
  {
    result = null;
    getLatestResult();
    //PBDash.putString(camera.getName() + "result", (result.toString()));
  }
}
