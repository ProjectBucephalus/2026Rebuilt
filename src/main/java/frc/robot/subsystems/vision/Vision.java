// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.function.Supplier;

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

  private int pipelineIndex = (int)SD.LL_EXPOSURE.defaultValue();

  private ArrayList<Double> rotationBuf = new ArrayList<Double>();
  private boolean rotationKnown = false;
  private boolean lastCycleRotationKnown = false;

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

  public void resetRotation() {rotationKnown = false;}

  @Override
  public void periodic() 
  {
    if (SD.LL_TOGGLE.get()) 
    {
      for (var ll : lls)
      {
        var rotationData = rotationDataSup.get();
        double heading = rotationData.getFirst();
        double omegaRps = rotationData.getSecond();

        var mt2 = ll.getMT2(heading);
        
        boolean useUpdate = !(mt2 == null || mt2.tagCount == 0 || omegaRps > 2.0);
        
        if (useUpdate) 
        {
          double stdDevFactor = Math.pow(mt2.avgTagDist, 2.0) / mt2.tagCount;

          double linearStdDev = linearStdDevBaseline * stdDevFactor;
          double rotStdDev = rotStdDevBaseline * stdDevFactor;

          estimateConsumer.accept(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds), VecBuilder.fill(linearStdDev, linearStdDev, rotStdDev));
        }
      }
    }

    if (!rotationKnown) 
    {
      lastCycleRotationKnown = false;

      for (var ll : lls) 
      {
        ll.getLimelightRotation().ifPresent
        (
          rotationReading ->
          {
            rotationBuf.add(0, rotationReading.getDegrees());
    
            if (rotationBuf.size() > mt1CyclesNeeded)
              {rotationBuf.remove(mt1CyclesNeeded);}
      
            if (rotationBuf.size() == mt1CyclesNeeded)
            {
              double lowest = rotationBuf.get(0).doubleValue();
              double highest = rotationBuf.get(0).doubleValue();
              
              for(var reading : rotationBuf)
              {
                lowest = Math.min(lowest, reading.doubleValue());
                highest = Math.max(highest, reading.doubleValue());
              }
              
              if (highest - lowest < 1)
              {
                rotationKnown = true;
                Robot.setYaw((highest + lowest) / 2);
              }
            }
          }
        );
      }

      if (!lastCycleRotationKnown) 
      {
        if (rotationKnown) 
        {
          rotationBuf.clear();
          lastCycleRotationKnown = true;
          //RobotContainer.s_Swerve.resetPose(new Pose2d(RobotContainer.swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble()))));
        }
      }
    }

    SmartDashboard.putBoolean("Rot Known", rotationKnown);
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
