// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.util.SD;
import static frc.robot.constants.Constants.Vision.*;

public class Vision extends SubsystemBase 
{
  public enum TagPOI {ALL, HUB, TOWER, OUTPOST, TRENCH}
  
  private final PoseEstimateConsumer estimateConsumer;
  private final Supplier<Pair<Double, Double>> rotationDataSup;
  private final Limelight[] lls;

  private EstimatedRobotPose mt2; // TODO rename to something more generic (same with anywhere else we talk about mt1/mt2)

  private int pipelineIndex = (int)SD.LL_EXPOSURE.defaultValue();

  /** Creates a new Vision. */
  public Vision(PoseEstimateConsumer estimateConsumer, Supplier<Pair<Double, Double>> rotationDataSup, Limelight... lls) 
  {
    this.estimateConsumer = estimateConsumer;
    this.rotationDataSup = rotationDataSup;
    this.lls = lls;
    setActivePOI(TagPOI.ALL);
  }

  public void setActivePOI(TagPOI activePOI) 
  {
    var validIDs = switch (activePOI) 
    {
      case ALL -> allIDs;
      case HUB -> hubIDs;
      case TOWER -> towerIDs;
      case OUTPOST -> outpostIDs;
      case TRENCH -> trenchIDs;
    };

    for (var ll : lls) ll.updateValidIDs(validIDs);
  }

  public void incrementPipeline() 
  {
    pipelineIndex = MathUtil.clamp(pipelineIndex + 1, 0, 7);
    for (var ll : lls) {ll.updatePipeline(pipelineIndex);}
    SD.LL_EXPOSURE.put((double)pipelineIndex);
  }

  public void decrementPipeline()
  {
    pipelineIndex = MathUtil.clamp(pipelineIndex - 1, 0, 7);
    for (var ll : lls) {ll.updatePipeline(pipelineIndex);}
    SD.LL_EXPOSURE.put((double)pipelineIndex);
  }

  @Override
  public void periodic() 
  {
    if (SD.LL_TOGGLE.get()) 
    {
      for (var ll : lls)
      {
        ll.periodic();

        // TODO bc we don't need heading, can rework this whole thing to just have the supplier directly give omegaRPS (if we even need that check?)
        var rotationData = rotationDataSup.get();
        double heading = rotationData.getFirst();
        double omegaRps = rotationData.getSecond();

        var est = ll.getPhotonEst();

        if (est.isPresent()) 
        {
          mt2 = est.get(); 
          boolean useUpdate = !(mt2.targetsUsed.size() != 0 && omegaRps > 2.0);
          
          if (useUpdate) 
          {
            double avgTagDist = 0;
            for (var target : mt2.targetsUsed)
              {avgTagDist += target.getBestCameraToTarget().getTranslation().getNorm();}
            // TODO Should divide avgTagDist by target count here so it is actually the avg and not the total

            double stdDevFactor = Math.pow((avgTagDist/mt2.targetsUsed.size()), 2.0) / mt2.targetsUsed.size();

            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double rotStdDev = rotStdDevBaseline * stdDevFactor;

            estimateConsumer.accept(mt2.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(mt2.timestampSeconds), VecBuilder.fill(linearStdDev, linearStdDev, rotStdDev));
          }
        }
      }
    }
  }

  @FunctionalInterface
  public static interface PoseEstimateConsumer 
  {
    public void accept
    (
      Pose2d visionRobotPoseMeters, 
      double timestampSeconds, 
      Matrix<N3, N1> visionMeasurementStdDevs
    );
  }
}
