package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot.ClimbPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

/** 
 * Heading-locked drive command with dynamic heading switching based on robot position 
 * @author 5985
 */
public class ClimbLockedDrive extends HeadingLockedDrive 
{
  private final Supplier<ClimbPosition> climbPosSup;
  
  /**
   * Creates a Heading-locked drive command to face the nearest station <p>
   * (From 2025 Reefscape, left as an example)
   * @param s_Swerve Drivebase subsystem
   * @param joystickSupplier XY translation input from joystick, [-1..1][-1..1]
   * @param rotationOffset Field relative rotation to treat as 0
   * @param robotPoseSup Supplier for robot XY position in field coordinates
   */
  public ClimbLockedDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Translation2d> joystickSupplier,
    Supplier<Pose2d> robotPoseSup,
    Supplier<ClimbPosition> climbPosSup
  ) 
  {
    super(s_Swerve, joystickSupplier, Rotation2d.kZero, Rotation2d.kZero, robotPoseSup);
    this.climbPosSup = climbPosSup;
  }

  @Override
  protected void updateTargetHeading()
  {
    var rotation = switch (climbPosSup.get()) 
    {
      case Left -> Rotation2d.kCCW_90deg;
      case Right -> Rotation2d.kCW_90deg;
    };
    targetHeading = FieldUtils.allianceRotateRotation(rotation);
  }
}