// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Conversions;
import frc.robot.util.SD;

/**
 * A drive command that prevents the robot from being within a given tolerance of cardinal-aligned
 */
public class NonCardinalDrive extends SwerveCommandBase 
{
  private final PIDController thetaController = new PIDController(0.02, Swerve.rotationKI, Swerve.rotationKD);

  protected DoubleSupplier rotationSup;
  protected double rotationVal;
  protected DoubleSupplier brakeSup;
  private final double tolerance;
  private final Supplier<Rotation2d> robotRotationSup;

  protected final SwerveRequest.FieldCentric driveRequest = new SwerveRequest
    .FieldCentric() 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /** Creates a new NonCardinalDrive. */
  public NonCardinalDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Rotation2d> robotRotationSup,
    Supplier<Translation2d> joystickSupplier, 
    DoubleSupplier rotationSup, 
    DoubleSupplier brakeSup, 
    double tolerance
  ) 
  {
    super(s_Swerve, joystickSupplier);
    this.rotationSup = rotationSup;
    this.brakeSup = brakeSup;
    this.tolerance = tolerance;
    this.robotRotationSup = robotRotationSup;
  }

  @Override
  public void execute()
  {
    motionXY = joystickSupplier.get();

    /* Get and process Rotation input */
    rotationVal = rotationSup.getAsDouble();

    double robotRotation = robotRotationSup.get().getDegrees();

    if (Math.abs(rotationVal) <= deadband) 
    {
      double wrappedRotation = Conversions.mod(robotRotation, 90);

      if (wrappedRotation < tolerance)
      {
        double error = tolerance - wrappedRotation;
        double targetRotation = robotRotation + error;
        rotationVal = thetaController.calculate(robotRotation, targetRotation);
      }
      else if (wrappedRotation > 90 - tolerance)
      {
        double error = wrappedRotation - (90 - tolerance);
        double targetRotation = robotRotation - error;
        rotationVal = thetaController.calculate(robotRotation, targetRotation);
      }
      else
        rotationVal = 0;
    }
    else
      {rotationVal *= MathUtil.interpolate(Control.maxRotThrottle, Control.minRotThrottle, brakeSup.getAsDouble());}

    if (motionXY.getX() != 0 || motionXY.getY() != 0)
      {SD.STATE_DRIVE.put("Manual");}

    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * Swerve.maxSpeed)
      .withVelocityY(motionXY.getY() * Swerve.maxSpeed)
      .withRotationalRate(rotationVal * Swerve.maxAngularVelocity)
    );
  }
}
