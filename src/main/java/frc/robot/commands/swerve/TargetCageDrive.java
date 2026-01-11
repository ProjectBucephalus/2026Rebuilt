package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetCageDrive extends HeadingLockedDrive
{
  public TargetCageDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Translation2d> joystickSupplier,
    Rotation2d targetHeading, 
    Rotation2d rotationOffset,
    Supplier<Translation2d> robotPosSup
  ) 
  {
    super(s_Swerve, joystickSupplier, targetHeading, rotationOffset, robotPosSup);
  }

  @Override
  protected void updateTargetHeading() 
  {
    rotationOffset = 
    MathUtil.isNear(robotXY.getX(), (FieldConstants.fieldCentre.getX()), Constants.Control.cageFaceDistance) ? 
      Rotation2d.kZero :
      Rotation2d.kCW_90deg;
  }
}
