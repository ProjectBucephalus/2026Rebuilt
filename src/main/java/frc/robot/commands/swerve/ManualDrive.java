package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.*;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PBDash;

/** 
 * Swerve drive interface for full manual control 
 * @author 5985
 */
public class ManualDrive extends SwerveCommandBase 
{
  protected DoubleSupplier rotationSup;
  protected double rotationVal;
  protected DoubleSupplier rotBrakeSup;

  protected final SwerveRequest.FieldCentric driveRequest = new SwerveRequest
    .FieldCentric() 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates a basic Manual drive command
   * @param s_Swerve Drivebase subsystem
   * @param joystickSupplier XY translation input from joystick, [-1..1][-1..1]
   * @param rotationSup Rotation input from joystick, [-1..1]
   * @param rotBrakeSup Brake axis input for rotation, [0..1]
   */
  public ManualDrive(CommandSwerveDrivetrain s_Swerve, Supplier<Translation2d> joystickSupplier, DoubleSupplier rotationSup, DoubleSupplier rotBrakeSup) 
  {
    super(s_Swerve, joystickSupplier);
    this.rotationSup = rotationSup;
    this.rotBrakeSup = rotBrakeSup;
  }

  @Override
  public void execute()
  {
    motionXY = joystickSupplier.get();

    /* Get and process Rotation input */
    rotationVal = rotationSup.getAsDouble();

    if (Math.abs(rotationVal) <= deadband) 
      {rotationVal = 0;}
    else
      {rotationVal *= MathUtil.interpolate(ControlConstants.maxRotThrottle, ControlConstants.minRotThrottle, rotBrakeSup.getAsDouble());}

    if (motionXY.getNorm() != 0)
      {PBDash.STATE_DRIVE.put("Manual");}

    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
      .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
      .withRotationalRate(rotationVal * SwerveConstants.maxAngularVelocity)
    );
  }
}
