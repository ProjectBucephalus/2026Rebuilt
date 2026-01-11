// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.util.libs.LimelightHelpers;
import frc.robot.util.libs.LimelightHelpers.PoseEstimate;

public class Limelight  
{    
  private final String name;
  
  /** Creates a new Limelight. */
  public Limelight(String name) 
    {this.name = name;}

  public void setIMUMode(int mode)
    {LimelightHelpers.SetIMUMode(name, mode);}

  public Optional<Rotation2d> getLimelightRotation()
  {
    var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

    return (mt1 != null && mt1.avgTagDist < 4) ? 
      Optional.of(mt1.pose.getRotation()) : 
      Optional.empty();
  }

  protected void updateValidIDs(int[] validIDs)
    {LimelightHelpers.SetFiducialIDFiltersOverride(name, validIDs);}

  protected void updatePipeline(int pipelineIndex)
    {LimelightHelpers.setPipelineIndex(name, pipelineIndex);}

  public PoseEstimate getMT2(double headingDeg)
  {
    LimelightHelpers.SetRobotOrientation(name, headingDeg, 0, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }

  public void periodic() {}
}
