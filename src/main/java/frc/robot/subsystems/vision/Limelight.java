// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.constants.Constants.Vision.trenchIDs;

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
   * 
   * @param name
   * @param robotToCamera Transform3d from the centre of the turret to the camera.
   */
  public Limelight(String name, Transform3d robotToCamera) 
    {
      this.camera = new PhotonCamera(name);
      structureToCamera = robotToCamera;
      photonEstimator = new PhotonPoseEstimator(kTagLayout, structureToCamera);
      onTurret = false;
    }

 // rotation2d supplier, translation2d assign in constructor + set flag to true (turret to robot)
  public Limelight(String name, Transform3d turretToCamera, Supplier<Rotation2d> turretAngleSup, Translation2d turretToRobot) 
  {
    this.camera = new PhotonCamera(name);
    this.turretAngleSup = turretAngleSup;
    this.turretToRobot = turretToRobot;
    structureToCamera = turretToCamera;
    photonEstimator = new PhotonPoseEstimator(kTagLayout, structureToCamera);
    onTurret = true;
  }

  public void getLatestResult() 
  {
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) 
      {result = results.get(results.size()-1);}
  }

  protected void updatePipeline(int pipelineIndex)
    {camera.setPipelineIndex(pipelineIndex);}

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

  public void periodic() 
  {
    getLatestResult();
    //SmartDashboard.putString(camera.getName() + "result", (result.toString()));
  }
}
