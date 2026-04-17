package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

/** 
 * Extension of Supplier<Translation2d> to rotate a Blue alliance pose to be Red alliance relative when needed 
 * @author 5985
 */
public class AllianceTranslation2d implements Supplier<Translation2d>
{
  private final Translation2d pointBlue;
  private final Translation2d  pointRed;

  /**
   * Constructs a new AllianceTranslation2d based on blue origin
   * @param x x-coordinate of Blue pose
   * @param y y-coordinate of Blue pose
   * @param rotation Rotation of Blue pose
   */
  public AllianceTranslation2d(double x, double y)
    {this(new Translation2d(x, y));}

  /**
   * Constructs a new AllianceTranslation2d based on blue origin
   * @param pose Blue alliance pose
   */
  public AllianceTranslation2d(Translation2d pose)
  {
    pointBlue = pose;
    pointRed = FieldUtils.rotateTranslation(pointBlue);
  }

  /** @return Blue rotated pose */
  public Translation2d blue() 
    {return pointBlue;}

  /** @return Red rotated pose */
  public Translation2d red() 
    {return pointRed;}

  /** @return Alliance rotated pose */
  @Override
  public Translation2d get() 
  {
    return switch (FieldUtils.getAlliance()) { case Blue -> pointBlue; case Red -> pointRed; };
  }
}
