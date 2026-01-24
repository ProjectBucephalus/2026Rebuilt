// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight  
{    
  private final PhotonCamera camera;
  // TODO should probably initialise photonEstimator in the constructor so that the tag layout and robotToCam transform can be just restricted to the constructor
  // Also, maybe let the robotToCam transform be provided in the constructor
  private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
  private Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  private final PhotonPoseEstimator photonEstimator;
  private PhotonPipelineResult result;

  /**
   * 
   * @param name
   * @param robotToCamera Transform3d from the centre of the robot to the camera, or from centre of a turret.
   */
  public Limelight(String name, Transform3d robotToCamera) 
    {
      this.camera = new PhotonCamera(name);
      kRobotToCam = robotToCamera;
      photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    }

  public void getLatestResult() 
  {
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) 
      {result = results.get(results.size()-1);}
  }

  protected void updateValidIDs(int[] validIDs) //TODO: Re-implement
  {
    
  }

  protected void updatePipeline(int pipelineIndex)
    {camera.setPipelineIndex(pipelineIndex);}

  public Optional<EstimatedRobotPose> getPhotonEst()
  { 
    if (result == null) return Optional.empty();

    var visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) 
    {
      visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
    }

    return visionEst;
  }

  public void periodic() 
  {
    getLatestResult();
    //SmartDashboard.putString(camera.getName() + "result", (result.toString()));
  }
}
