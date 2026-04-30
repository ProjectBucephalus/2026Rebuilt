package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PBDash;

import static frc.robot.constants.Constants.VisionConstants.*;

/** 
 * Computer-vision localisation master-system to manage multiple photon or limelight cameras 
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Vision extends SubsystemBase 
{
  @FunctionalInterface
  public interface PoseEstimateConsumer 
  {
    public void accept
    (
      Pose2d visionRobotPoseMeters, 
      double timestampSeconds, 
      Matrix<N3, N1> visionMeasurementStdDevs
    );
  }

  private final Limelight[] lls;
  private final Supplier<Double> rpsSup;
  private final PoseEstimateConsumer estimateConsumer;
  /** Timestamp of last good pose estimate, seconds, -1 on initialisation */
  @Logged
  double lastGoodPose = -1; 
  /** True if IO_LL was true last cycle */
  boolean usingVision = true;

  /**
   * Creates a vision master-system to manage the provided cameras
   * @param estimateConsumer Link into drivebase to update localisation
   * @param rpsSup Supplier for robot rate of rotation, radians per second
   * @param lls List of limelight or photon cameras
   */
  public Vision(PoseEstimateConsumer estimateConsumer, Supplier<Double> rpsSup, Limelight... lls) 
  {
    this.lls = lls;
    this.rpsSup = rpsSup;
    this.estimateConsumer = estimateConsumer;
  }

  /** 
   * @return {@code true} if localisation can be trusted (or simulated)
   * <li>    {@code false} if running on odometry only
   */
  @Logged
  public boolean hasLocalisation()
    {return (RobotBase.isSimulation() || (lastGoodPose > -1 && Timer.getTimestamp() - lastGoodPose < visionFrequencyThreshold));}

  /**
   * Accepts a given robot pose as if it were a valid localisation estimate
   * @param pose Robot pose in field space, ignores rotation
   */
  public void setPose(Pose2d pose)
  {
    lastGoodPose = Timer.getTimestamp();
    estimateConsumer.accept(pose, Utils.getCurrentTimeSeconds(), VecBuilder.fill(0, 0, Double.POSITIVE_INFINITY));
  }

  @Override
  public void periodic() 
  {
    if (PBDash.IO_LL.get())
    {
      usingVision = true;

      for (var ll : lls)
      {
        // Skip this limelight if it isn't active
        
        if (ll.isActive())
        {
          ll.getPhotonEst().ifPresent(est -> {
            // Reject update if it contains no tags, or if the robot is rotating too fast
            if (est.targetsUsed.isEmpty()|| Math.abs(rpsSup.get()) >= 2.0) return;

            double accTagDist = 0;
            for (var target : est.targetsUsed) accTagDist += target.getBestCameraToTarget().getTranslation().getNorm();
            double avgTagDist = accTagDist / est.targetsUsed.size();

            // The more tags seen and the closer we are on average to them, the more trustworthy the estimate is
            // If this is the first time we've seen tags since last losing localisation, we trust the estimate fully
            double stdDevFactor = Math.pow(avgTagDist, 2.0) / est.targetsUsed.size();
            double linearStdDev = hasLocalisation() ? linearStdDevBaseline * stdDevFactor : 0;
            double rotStdDev = hasLocalisation() ? rotStdDevBaseline * stdDevFactor : 0;

            double timestamp = Utils.fpgaToCurrentTime(est.timestampSeconds);

            PBDash.putFieldObject("Raw Pose" + ll.getName(), est.estimatedPose.toPose2d());
            Pose2d poseOut = est.estimatedPose.toPose2d().transformBy(ll.getCameraToStructure());
            // If the camera is mounted on a turret, apply additional offset processing
            if (ll.isOnTurret()) poseOut = poseOut.transformBy(ll.getTurretToRobot(timestamp));
            //PBDash.putString("Processed Pose" + ll.getName(), poseOut.toString());

            // Update time since last good pose estimate
            lastGoodPose = Timer.getTimestamp();

            // Send pose estimate to consumer
            estimateConsumer.accept(poseOut, timestamp, VecBuilder.fill(linearStdDev, linearStdDev, rotStdDev));


          });
        }
      } 
    }
    else if (usingVision)
    {
      usingVision = false;
      lastGoodPose = -1;
    }
  }
}
