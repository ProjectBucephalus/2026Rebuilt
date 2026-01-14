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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFollowDrive extends Command 
{
  private final Supplier<SwerveDriveState> swerveStateSup;
  private final CommandSwerveDrivetrain s_Swerve;
  private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();    

  private final ArrayList<Pose2d> waypoints;
  private final ArrayList<Double> radiusPerSegment;

  private boolean onPath = false;
  private int currentWaypoint = 0;

  /** Creates a new PathFollowDrive. */
  public PathFollowDrive(CommandSwerveDrivetrain s_Swerve, Supplier<SwerveDriveState> swerveStateSup, double pointRadius, Pose2d[] targetSequence)
  {
    this.swerveStateSup = swerveStateSup;
    this.s_Swerve = s_Swerve;
    this.waypoints = new ArrayList<>(targetSequence.length * 3 - 2);
    this.radiusPerSegment = new ArrayList<>(targetSequence.length - 1);

    for (int i = 0; i < targetSequence.length - 1; i++) 
    {
      final var current = targetSequence[i];
      final var next = targetSequence[i + 1];

      final var pathSegment = new Transform2d(current, next);

      final double segmentLength = pathSegment.getTranslation().getNorm();
      final double waypointDist = Conversions.clamp(pointRadius, 0, segmentLength / 3);
      final double lengthRatio = waypointDist / segmentLength;

      final var waypointTransform = pathSegment.times(lengthRatio);

      final var waypoint1 = current.plus(waypointTransform);
      final var waypoint2 = next.plus(waypointTransform.inverse());

      radiusPerSegment.add(waypointDist);
      waypoints.addAll(Arrays.asList(new Pose2d[] {current, waypoint1, waypoint2}));
    }

    radiusPerSegment.add(0.0);
    waypoints.add(targetSequence[targetSequence.length - 1]);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
    {currentWaypoint = 0;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    final var pose = swerveStateSup.get().Pose;

    final var targetIndex = Math.min(onPath ? currentWaypoint + 1 : currentWaypoint, waypoints.size() - 1);
    
    final var target = waypoints.get(targetIndex);
    
    s_Swerve.setControl
    (
      driveRequest.withSpeeds
      (
        s_Swerve.calculateDrivePID(target, pose)
      )
    );
        
    final var currentSegment = Math.floorDiv(currentWaypoint, 3);
    final double targetDist = radiusPerSegment.get(currentSegment);

    if (FieldUtils.nearPose(pose, target, targetDist)) {
      currentWaypoint++;
      onPath = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentWaypoint == waypoints.size();
  }
}
