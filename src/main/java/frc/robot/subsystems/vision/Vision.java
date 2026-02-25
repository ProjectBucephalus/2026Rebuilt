package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.PBDash;
import static frc.robot.constants.Constants.VisionConstants.*;

/** 
 * Computer-vision localisation master-system to manage multiple photon or limelight cameras 
 * @author 5985
 */
public class Vision extends SubsystemBase 
{
  private final PoseEstimateConsumer estimateConsumer;
  private final Supplier<Double> rpsSup;
  private final Limelight[] lls;
  /** Timestamp of last good pose estimate, seconds, -1 on initialisation */
  @Logged
  double lastGoodPose = -1; 
  /** Time since last good pose estimate, seconds */ // TODO Remove this or lastGoodPose? don't need same measurement in two perspectives
  @Logged
  double timeSince = 0; 
  /** True only while there is a recent valid pose estimate */
  boolean haveLocalisation = false;
  @Logged
  boolean usingVision = true;

  private int pipelineIndex = PBDash.LL_EXPOSURE.defaultVal();

  /**
   * Creates a vision master-system to manage the provided cameras
   * @param estimateConsumer Link into drivebase to update localisation
   * @param rpsSup Supplier for robot rate of rotation, radians per second
   * @param lls List of limelight or photon cameras
   */
  public Vision(PoseEstimateConsumer estimateConsumer, Supplier<Double> rpsSup, Limelight... lls) 
  {
    this.estimateConsumer = estimateConsumer;
    this.rpsSup = rpsSup;
    this.lls = lls;
  }

  /** Increments all camera pipelines in range [0..7] */
  public void incrementPipeline() 
  {
    pipelineIndex = MathUtil.clamp(pipelineIndex + 1, 0, 7);
    for (var ll : lls) {ll.updatePipeline(pipelineIndex);}
    PBDash.LL_EXPOSURE.put(pipelineIndex);
  }

  /** Decrements all camera pipelines in range [0..7] */
  public void decrementPipeline()
  {
    pipelineIndex = MathUtil.clamp(pipelineIndex - 1, 0, 7);
    for (var ll : lls) {ll.updatePipeline(pipelineIndex);}
    PBDash.LL_EXPOSURE.put(pipelineIndex);
  }

  /** 
   * @return {@code true} if localisation can be trusted (or simulated)
   * <li>    {@code false} if running on odometry only
   */
  @Logged
  public boolean hasLocalisation()
  {
    return haveLocalisation || Robot.isSimulation();
  }

  /**
   * Accepts a given robot pose as if it were a valid localisation estimate
   * @param pose Robot pose in field space, ignores rotation
   */
  public void setPose(Pose2d pose)
  {
    lastGoodPose = Timer.getTimestamp();
    timeSince = 0;
    haveLocalisation = true;

    estimateConsumer.accept(pose, Utils.getCurrentTimeSeconds(), VecBuilder.fill(0, 0, 1000));
  }

  @Override
  public void periodic() 
  {
    if (PBDash.LL_TOGGLE.get()) 
    {
      for (var ll : lls)
      {
        usingVision = true;
        ll.update();

        // Pose estimate returns Optional, so may or may not be present
        ll.getPhotonEst().ifPresent(est -> {
          // Reject update if it contains no tags, or if the robot is rotating too fast         
          if (est.targetsUsed.size() != 0 && Math.abs(rpsSup.get()) < 2.0) 
          {
            double avgTagDist = 
              est.targetsUsed
                 .stream()
                 .collect(Collectors.averagingDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm()));

            // The more tags seen and the closer we are on average to them, the more trustworthy the estimate is
            double stdDevFactor = Math.pow(avgTagDist, 2.0) / est.targetsUsed.size();
            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double rotStdDev = rotStdDevBaseline * stdDevFactor;

            // If the camera is mounted on a turret, apply additional offset processing
            var poseOut = 
              ll.isOnTurret() 
              ? est.estimatedPose.toPose2d().transformBy(ll.getTurretToRobot())
              : est.estimatedPose.toPose2d();
            
            // Update time since last good pose estimate
            lastGoodPose = Timer.getTimestamp();
            timeSince = 0;
            haveLocalisation = true;

            // Send pose estimate to consumer
            estimateConsumer.accept(poseOut, Utils.fpgaToCurrentTime(est.timestampSeconds), VecBuilder.fill(linearStdDev, linearStdDev, rotStdDev));
          }
        });
      }
    } 
    else 
    {
      if (usingVision)
      {
        usingVision = false;
        haveLocalisation = false;
        lastGoodPose = -1;
      }
    }

    // If no valid pose is found, update time since last pose
    timeSince = Timer.getTimestamp() - lastGoodPose;
    if (lastGoodPose == -1 || timeSince >= visionFrequencyThreshold) 
      haveLocalisation = false; 
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
