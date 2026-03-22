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

/** 
 * A drive command for rotating about a point other than robot-centre 
 * @author 5985
 */
public class OffsetDrive extends SwerveCommandBase 
{
  protected DoubleSupplier rotationSup;
  protected double rotationVal;
  protected DoubleSupplier brakeSup;

  protected final SwerveRequest.FieldCentric driveRequest = new SwerveRequest
    .FieldCentric() 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates a new OffsetDrive
   * @param s_Swerve Swerve subsystem
   * @param joystickSupplier Joystick translation input, T2d [-1..1]
   * @param rotationSup Joystick rotation input, [-1..1]
   * @param brakeSup Brake axis input, [0..1]
   * @param centreOffset Offset from centre of drivebase to centre of rotation, metres Fore/Port
   */
  public OffsetDrive(CommandSwerveDrivetrain s_Swerve, Supplier<Translation2d> joystickSupplier, DoubleSupplier rotationSup, DoubleSupplier brakeSup, Translation2d centreOffset) 
  {
    super(s_Swerve, joystickSupplier);
    this.rotationSup = rotationSup;
    this.brakeSup = brakeSup;
    driveRequest.withCenterOfRotation(centreOffset);
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
      {rotationVal *= MathUtil.interpolate(ControlConstants.maxRotThrottle, ControlConstants.minRotThrottle, brakeSup.getAsDouble());}

    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * SwerveConstants.maxSpeed)
      .withVelocityY(motionXY.getY() * SwerveConstants.maxSpeed)
      .withRotationalRate(rotationVal * SwerveConstants.maxAngularVelocity)
    );
  }
}
