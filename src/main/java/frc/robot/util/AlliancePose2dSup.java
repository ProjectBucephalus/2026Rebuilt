package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

public class AlliancePose2dSup implements Supplier<Pose2d>
{
  private final Pose2d poseBlue;

  /**
   * Constructs a new AlliancePose2dSup based on blue origin
   * @param x
   * @param y
   * @param rotation
   */
  public AlliancePose2dSup(double x, double y, double rotation)
    {poseBlue = Conversions.buildPose(x, y, rotation);}

  public AlliancePose2dSup(Translation2d translation, Rotation2d rotation)
    {poseBlue = new Pose2d(translation, rotation);}

  public AlliancePose2dSup(Pose2d pose)
    {poseBlue = pose;}

  @Override
  public Pose2d get() 
  {
    return FieldUtils.isRedAlliance() ? poseBlue.rotateAround(FieldConstants.fieldCentre, Rotation2d.k180deg) : poseBlue;
  }
}
