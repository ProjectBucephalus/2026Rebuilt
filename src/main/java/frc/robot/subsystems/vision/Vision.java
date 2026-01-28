// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.SD;
import static frc.robot.constants.Constants.Vision.*;

public class Vision extends SubsystemBase 
{
  public enum TagPOI {ALL, HUB, TOWER, OUTPOST, TRENCH}
  
  private final PoseEstimateConsumer estimateConsumer;
  private final Supplier<Double> rpsSup;
  private final Limelight[] lls;
  private boolean onTurret;

  private Pose2d poseOut = Pose2d.kZero;

  private int pipelineIndex = (int)SD.LL_EXPOSURE.defaultValue();

  /** Creates a new Vision. */
  public Vision(PoseEstimateConsumer estimateConsumer, Supplier<Double> rpsSup, Limelight... lls) 
  {
    this.estimateConsumer = estimateConsumer;
    this.rpsSup = rpsSup;
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

  // TODO Additional processing is required to account for cameras mounted on turrets
  @Override
  public void periodic() 
  {
    if (SD.LL_TOGGLE.get()) 
    {
      for (var ll : lls)
      {
        ll.periodic();

        var maybeEst = ll.getPhotonEst();
        onTurret = ll.isOnTurret();

        if (maybeEst.isPresent()) 
        {
          var est = maybeEst.get(); 
          boolean useUpdate = (est.targetsUsed.size() != 0 && rpsSup.get() < 2.0);
          
          if (useUpdate) 
          {
            double avgTagDist = 0;
            for (var target : est.targetsUsed)
              {avgTagDist += target.getBestCameraToTarget().getTranslation().getNorm();}


            double stdDevFactor = Math.pow((avgTagDist/est.targetsUsed.size()), 2.0) / est.targetsUsed.size();

            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double rotStdDev = rotStdDevBaseline * stdDevFactor;

            
            if (onTurret == true)
              poseOut = est.estimatedPose.toPose2d().transformBy(new Transform2d(ll.getTurretToRobot(), ll.getTurretAngle().unaryMinus()));
            else 
              poseOut = est.estimatedPose.toPose2d();

            estimateConsumer.accept(poseOut, Utils.fpgaToCurrentTime(est.timestampSeconds), VecBuilder.fill(linearStdDev, linearStdDev, rotStdDev));
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
