package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

public class TargetProcessorDrive extends HeadingLockedDrive 
{
  /** Creates a new TargetProcessorDrive. */
  public TargetProcessorDrive
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
    targetHeading = FieldUtils.isRedAlliance() ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg;
  }
}
