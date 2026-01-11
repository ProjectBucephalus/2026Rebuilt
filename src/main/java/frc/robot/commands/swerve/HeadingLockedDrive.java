package frc.robot.commands.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.SD;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
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

  /** Creates a new ManualDrive. */
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

    rotationKP = Swerve.rotationKP;
    rotationKI = Swerve.rotationKI;
    rotationKD = Swerve.rotationKD;

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
      {SD.STATE_DRIVE.put("Heading Locked");}

    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * Swerve.maxSpeed)
      .withVelocityY(motionXY.getY() * Swerve.maxSpeed)
      .withTargetDirection(targetHeading.plus(rotationOffset))
      .withHeadingPID(rotationKP, rotationKI, rotationKD)
    );
  }

  /** Processing to dynamicaly update the target heading */
  protected void updateTargetHeading() {}

  /** Processing to dynamicaly update the heading PID */
  protected void updateRotationPID()
  {
    rotationKP = Swerve.rotationKP;
    rotationKI = Swerve.rotationKI;
    rotationKD = Swerve.rotationKD;
  }
}
