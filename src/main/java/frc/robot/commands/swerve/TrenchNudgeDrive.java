package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;

/** 
 * A drive command that prevents the robot from being within a given tolerance of cardinal-aligned 
 * @author 5985
 */
public class TrenchNudgeDrive extends SwerveCommandBase 
{
  private final PIDController thetaController = new PIDController(0.02, SwerveConstants.rotationKI, SwerveConstants.rotationKD);

  protected DoubleSupplier rotationSup;
  protected double rotationVal;
  protected DoubleSupplier brakeSup;
  private final Supplier<Rotation2d> robotRotationSup;

  protected final SwerveRequest.FieldCentric driveRequest = new SwerveRequest
    .FieldCentric() 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates a basic Manual drive command
   * @param s_Swerve Drivebase subsystem
   * @param joystickSupplier XY translation input from joystick, [-1..1][-1..1]
   * @param rotationSup Rotation input from joystick, [-1..1]
   * @param brakeSup Brake axis input for rotation, [0..1]
   * @param robotRotationSup Supplier for current robot rotation
   */
  public TrenchNudgeDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Translation2d> joystickSupplier, 
    DoubleSupplier rotationSup, 
    DoubleSupplier brakeSup, 
    Supplier<Rotation2d> robotRotationSup
  ) 
  {
    super(s_Swerve, joystickSupplier);
    this.rotationSup = rotationSup;
    this.brakeSup = brakeSup;
    this.robotRotationSup = robotRotationSup;
  }

  @Override
  public void execute()
  {
    motionXY = joystickSupplier.get();

    double robotRotation = robotRotationSup.get().getDegrees();

    /* Get and process Rotation input */
    rotationVal = rotationSup.getAsDouble();

    // Rotation stick not being actively controlled
    if (Math.abs(rotationVal) <= deadband) 
    {
      // Wrap the robot's rotation to [0..180) (effectively, clockwise degrees past previous straight) 
      double wrappedRotation = Conversions.mod(robotRotation, 180);

      rotationVal = 
      wrappedRotation > 90 ?
      // If we're more than halfway to the next straight, rotate to it
      thetaController.calculate(robotRotation, robotRotation + (180 - wrappedRotation)) :
      // Less than halfway to next straight, rotate to previous straight
      thetaController.calculate(robotRotation, robotRotation - wrappedRotation);
    }
    else
      {rotationVal *= MathUtil.interpolate(ControlConstants.maxRotThrottle, ControlConstants.minRotThrottle, brakeSup.getAsDouble());}

    if (motionXY.getX() != 0 || motionXY.getY() != 0)
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
