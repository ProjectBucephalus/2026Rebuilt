package frc.robot.commands.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PBDash;

/** Swerve drive interface to have the robot face a fixed direction */
public class HeadingLockedDrive extends SwerveCommandBase 
{
  protected Rotation2d rotationOffset;
  protected Rotation2d targetHeading;

  protected double rotationKP;
  protected double rotationKI;
  protected double rotationKD;

  protected Supplier<Translation2d> robotPosSup;

  protected final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest
    .FieldCentricFacingAngle()
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates a basic Heading-locked drive command
   * @param s_Swerve Drivebase subsystem
   * @param joystickSupplier XY translation input from joystick, [-1..1][-1..1]
   * @param targetHeading Heading for robot to face relative to Offset
   * @param rotationOffset Field relative rotation to treat as 0
   * @param robotPosSup Supplier for robot XY position in field coordinates
   */
  public HeadingLockedDrive
  (
    CommandSwerveDrivetrain s_Swerve,  
    Supplier<Translation2d> joystickSupplier,
    Rotation2d targetHeading, 
    Rotation2d rotationOffset,
    Supplier<Translation2d> robotPosSup
  ) 
  {
    super(s_Swerve, joystickSupplier);

    rotationKP = SwerveConstants.rotationKP;
    rotationKI = SwerveConstants.rotationKI;
    rotationKD = SwerveConstants.rotationKD;

    driveRequest.HeadingController.setPID(rotationKP, rotationKI, rotationKD);

    this.targetHeading = targetHeading;
    this.rotationOffset = rotationOffset;
    this.robotPosSup = robotPosSup;
  }

  @Override
  public void execute()
  {
    motionXY = joystickSupplier.get();
    robotXY = robotPosSup.get();

    updateTargetHeading();
    updateRotationPID();

    if (motionXY.getNorm() != 0)
      {PBDash.STATE_DRIVE.put("Heading Locked");}

    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
      .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
      .withTargetDirection(targetHeading.plus(rotationOffset))
      .withHeadingPID(rotationKP, rotationKI, rotationKD)
    );
  }

  /** Processing to dynamicaly update the target heading */
  protected void updateTargetHeading() {}

  /** Processing to dynamicaly update the heading PID */
  protected void updateRotationPID()
  {
    rotationKP = SwerveConstants.rotationKP;
    rotationKI = SwerveConstants.rotationKI;
    rotationKD = SwerveConstants.rotationKD;
  }
}
