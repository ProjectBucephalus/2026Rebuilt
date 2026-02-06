// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.SD;
import static frc.robot.constants.Constants.Vision.*;

public class Vision extends SubsystemBase 
{
 
  private final PoseEstimateConsumer estimateConsumer;
  private final Supplier<Double> rpsSup;
  private final Limelight[] lls;
  /** Timestamp of last good pose estimate, seconds */
  double lastGoodPose = -1; 
  /** Time since last good pose estimate, seconds */
  double timeSince = 0; 
  boolean havePoseFromVision = false;

  private int pipelineIndex = (int)SD.LL_EXPOSURE.defaultValue();

  /** Creates a new Vision. */
  public Vision(PoseEstimateConsumer estimateConsumer, Supplier<Double> rpsSup, Limelight... lls) 
  {
    this.estimateConsumer = estimateConsumer;
    this.rpsSup = rpsSup;
    this.lls = lls;
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

  public boolean getPoseFromVision()
  {
    return havePoseFromVision;
  }

  @Override
  public void periodic() 
  {
    if (SD.LL_TOGGLE.get()) 
    {
      for (var ll : lls)
      {
        ll.periodic();

        var maybeEst = ll.getPhotonEst();

        if (maybeEst.isPresent()) 
        {
          var est = maybeEst.get(); 
          boolean useUpdate = (est.targetsUsed.size() != 0 && rpsSup.get() < 2.0);
          
          if (useUpdate) 
          {
            double avgTagDist = est
              .targetsUsed
              .stream()
              .collect(Collectors.averagingDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm()));

            double stdDevFactor = Math.pow(avgTagDist, 2.0) / est.targetsUsed.size();

            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double rotStdDev = rotStdDevBaseline * stdDevFactor;

            var poseOut = 
              ll.isOnTurret() 
              ? est.estimatedPose.toPose2d().transformBy(new Transform2d(ll.getTurretToRobot(), ll.getTurretAngle().unaryMinus()))
              : est.estimatedPose.toPose2d();

            lastGoodPose = Timer.getTimestamp();
            timeSince = 0;
            havePoseFromVision = true;

            estimateConsumer.accept(poseOut, Utils.fpgaToCurrentTime(est.timestampSeconds), VecBuilder.fill(linearStdDev, linearStdDev, rotStdDev));
          }
        } else 
        {
          timeSince = Timer.getTimestamp() - lastGoodPose;
          if (lastGoodPose == -1 || timeSince >= Constants.Vision.visionFrequencyThreshold) 
          {
            havePoseFromVision = false; 
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
