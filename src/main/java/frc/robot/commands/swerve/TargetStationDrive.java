package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Heading-locked drive command with dynamic heading switching based on robot position */
public class TargetStationDrive extends HeadingLockedDrive 
{
  /**
   * Creates a Heading-locked drive command to face the nearest station <p>
   * (From 2025 Reefscape, left as an example)
   * @param s_Swerve Drivebase subsystem
   * @param joystickSupplier XY translation input from joystick, [-1..1][-1..1]
   * @param rotationOffset Field relative rotation to treat as 0
   * @param robotPoseSup Supplier for robot XY position in field coordinates
   */
  public TargetStationDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Translation2d> joystickSupplier,
    Rotation2d rotationOffset,
    Supplier<Pose2d> robotPoseSup
  ) 
  {
    super(s_Swerve, joystickSupplier, Rotation2d.kZero, rotationOffset, robotPoseSup);
  }

  @Override
  protected void updateTargetHeading()
  {
    targetHeading = redAlliance ^ (robotPose.getY() >= 4.026) ? 
      new Rotation2d(Units.degreesToRadians(-55)) : // Left side if blue, right side if red
      new Rotation2d(Units.degreesToRadians(55)); // Right side if blue, left side if red
  }
}