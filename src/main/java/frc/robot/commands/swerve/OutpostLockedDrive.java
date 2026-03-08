package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

/** 
 * Heading-locked drive command with dynamic heading switching based on robot position 
 * @author 5985
 */
public class OutpostLockedDrive extends HeadingLockedDrive 
{
  /**
   * Creates a Heading-locked drive command to face the nearest station <p>
   * (From 2025 Reefscape, left as an example)
   * @param s_Swerve Drivebase subsystem
   * @param joystickSupplier XY translation input from joystick, [-1..1][-1..1]
   * @param rotationOffset Field relative rotation to treat as 0
   * @param robotPoseSup Supplier for robot XY position in field coordinates
   */
  public OutpostLockedDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Translation2d> joystickSupplier,
    Supplier<Pose2d> robotPoseSup
  ) 
  {
    super(s_Swerve, joystickSupplier, Rotation2d.kZero, Rotation2d.kZero, robotPoseSup);
  }

  @Override
  public void initialize() 
  {
    var currRotation = robotPose.getRotation().getDegrees();
    double diff90 = Math.abs(currRotation - 90);
    double diff180 = Math.abs(currRotation - 180);

    var targetRotation = diff90 < diff180 ? Rotation2d.kCW_90deg : Rotation2d.k180deg;
    targetHeading = FieldUtils.allianceRotateRotation(targetRotation);
  }
}