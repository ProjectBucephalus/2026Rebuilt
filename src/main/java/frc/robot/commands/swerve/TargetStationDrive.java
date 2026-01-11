package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetStationDrive extends HeadingLockedDrive 
{
  /** Creates a new TargetStationDrive. */
  public TargetStationDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Translation2d> joystickSupplier,
    Rotation2d rotationOffset,
    Supplier<Translation2d> robotPosSup
  ) 
  {
    super(s_Swerve, joystickSupplier, Rotation2d.kZero, rotationOffset, robotPosSup);
  }

  @Override
  protected void updateTargetHeading()
  {
    targetHeading = redAlliance ^ (robotXY.getY() >= 4.026) ? 
      new Rotation2d(Units.degreesToRadians(-55)) : // Left side if blue, right side if red
      new Rotation2d(Units.degreesToRadians(55)); // Right side if blue, left side if red
  }
}