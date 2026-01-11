// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFollowDrive extends Command 
{
  private final Supplier<SwerveDriveState> swerveStateSup;
  private final CommandSwerveDrivetrain s_Swerve;
  private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();    

  private boolean onPath = false;

  /*
    public Command poseDriveCommand(Supplier<Pose2d> targetSupplier, Supplier<SwerveDriveState> swerveStateSup) 
    {
      final var driveRequest = new SwerveRequest.ApplyRobotSpeeds();    

      return 
      run
      (() -> {
        final Pose2d pose = swerveStateSup.get().Pose;
        final Pose2d target = targetSupplier.get();

        setControl
        (
          driveRequest.withSpeeds
          (
            calculateDrivePID(target, pose)
          )
        );
      }).until(() -> FieldUtils.atPose(swerveStateSup.get().Pose, targetSupplier.get()));
    }
  */

  private ArrayList<Pose2d> waypoints;
  /** Creates a new PathFollowDrive. */
  public PathFollowDrive(CommandSwerveDrivetrain s_Swerve, Supplier<SwerveDriveState> swerveStateSup, double pointRadius, Pose2d... targetSequence)
  {
    this.swerveStateSup = swerveStateSup;
    this.s_Swerve = s_Swerve;
    this.waypoints = new ArrayList<>(targetSequence.length * 3 - 2);

    waypoints.add(targetSequence[targetSequence.length - 1]);

    for (int i = targetSequence.length - 1; i > 0 ; i--) {
      var current = targetSequence[i - 1];
      var next = targetSequence[i];

      var transform = new Transform2d(current, next);

      double length = transform.getTranslation().getNorm();
      double lengthRatio = Conversions.clamp(pointRadius, 0, length / 3) / length;

      var clampedTransform = transform.times(lengthRatio);

      var intermediate1 = current.plus(clampedTransform);
      var intermediate2 = next.plus(clampedTransform.inverse());

      waypoints.addAll(Arrays.asList(new Pose2d[] {intermediate2, intermediate1, current}));
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d pose = swerveStateSup.get().Pose;
    final int lastIndex = waypoints.size() - 1;

    if (!onPath) {
      final var target = waypoints.get(lastIndex);

      s_Swerve.setControl
      (
        driveRequest.withSpeeds
        (
          s_Swerve.calculateDrivePID(target, pose)
        )
      );

      if (FieldUtils.atPose(pose, target)) 
        onPath = true;
    } else {
      final var target = waypoints.get(lastIndex - 1);

      s_Swerve.setControl
      (
        driveRequest.withSpeeds
        (
          s_Swerve.calculateDrivePID(target, pose)
        )
      );

      final double dist = target.getTranslation().minus(waypoints.get(lastIndex).getTranslation()).getNorm();

      if (FieldUtils.nearPose(pose, target, dist)) 
        waypoints.remove(lastIndex);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return waypoints.isEmpty();
  }
}
