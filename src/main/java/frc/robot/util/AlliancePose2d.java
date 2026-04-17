package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** 
 * Extension of Supplier<Pose2d> to rotate a Blue alliance pose to be Red alliance relative when needed 
 * @author 5985
 */
public class AlliancePose2d implements Supplier<Pose2d>
{
  private final Pose2d poseBlue;
  private final Pose2d  poseRed;

  /**
   * Constructs a new AlliancePose2d based on blue origin
   * @param x x-coordinate of Blue pose
   * @param y y-coordinate of Blue pose
   * @param rotation Rotation of Blue pose
   */
  public AlliancePose2d(double x, double y, double rotation)
  {
    this(Conversions.buildPose(x, y, rotation));
  }

  /**
   * Constructs a new AlliancePose2d based on blue origin
   * @param translation Translation of Blue pose
   * @param rotation Rotation of Blue pose
   */
  public AlliancePose2d(Translation2d translation, Rotation2d rotation)
  {
    this(new Pose2d(translation, rotation));
  }

  /**
   * Constructs a new AlliancePose2d based on blue origin
   * @param pose Blue alliance pose
   */
  public AlliancePose2d(Pose2d pose)
  {
    poseBlue = pose;
    poseRed = FieldUtils.rotatePose(poseBlue);
  }

  /** @return Blue rotated pose */
  public Pose2d blue() 
    {return poseBlue;}

  /** @return Red rotated pose */
  public Pose2d red() 
    {return poseRed;}

  /** @return Alliance rotated pose */
  @Override
  public Pose2d get() 
  {
    return switch (FieldUtils.getAlliance()) { case Blue -> poseBlue; case Red -> poseRed; };
  }
}
