// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.*;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PBDash;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
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
      {rotationVal *= MathUtil.interpolate(Control.maxRotThrottle, Control.minRotThrottle, brakeSup.getAsDouble());}

    if (motionXY.getNorm() != 0)
      {PBDash.STATE_DRIVE.put("Manual");}

    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * Swerve.maxSpeed)
      .withVelocityY(motionXY.getY() * Swerve.maxSpeed)
      .withRotationalRate(rotationVal * Swerve.maxAngularVelocity)
    );
  }
}
