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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Pathfinding.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Conversions;

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

  private Pose2d robotPose;

  /**
   * Creates a new PathFollowDrive to follow the given sequence
   * @param s_Swerve        Swervedrive subsystem
   * @param swerveStateSup  Swerve state supplier from Robot to avoid expensive calls to the swerve system
   * @param pointRadius     Approach distance before switching to next point, metres
   * @param targetRotation  Rotation for robot to face, applies over entire path
   * @param targetSequence  List of Translation2d to navigate through, start to end
   */
  public PathFollowDrive(CommandSwerveDrivetrain s_Swerve, Supplier<SwerveDriveState> swerveStateSup, Path path)
  {
    this.swerveStateSup = swerveStateSup;
    this.s_Swerve = s_Swerve;

    this.waypoints = new ArrayList<>(path.sequence().length * 3 - 2);
    this.radiusPerSegment = new ArrayList<>(path.sequence().length - 1);

    for (int i = 0; i < path.sequence().length - 1; i++) 
    {
      final var current = path.sequence()[i];
      final var next = path.sequence()[i + 1];

      final double segmentLength = current.getDistance(next);
      final double waypointDist = Conversions.clamp(path.pointRadius(), 0, segmentLength / 3);
      final double lengthRatio = waypointDist / segmentLength;

      final var waypoint1 = current.interpolate(next, lengthRatio);
      final var waypoint2 = next.interpolate(current, lengthRatio);

      radiusPerSegment.add(waypointDist);
      waypoints.addAll
      (
        Arrays.asList
        (
          new Pose2d(current, path.heading()), 
          new Pose2d(waypoint1, path.heading()), 
          new Pose2d(waypoint2, path.heading())
        )
      );
    }

    radiusPerSegment.add(0.0);
    waypoints.add(new Pose2d(path.sequence()[path.sequence().length - 1], path.heading()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    currentWaypoint = 0;
    onPath = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    robotPose = swerveStateSup.get().Pose;

    final var targetIndex = Math.min(onPath ? currentWaypoint + 1 : currentWaypoint, waypoints.size() - 1);
    
    final var targetPose = waypoints.get(targetIndex);
    
    s_Swerve.setControl
    (
      driveRequest.withSpeeds
      (
        s_Swerve.calculateDrivePID(targetPose, robotPose)
      )
    );
        
    final var currentSegment = Math.floorDiv(currentWaypoint, 3);
    final double targetDist = radiusPerSegment.get(currentSegment);

    if (Conversions.nearTranslation(robotPose.getTranslation(), targetPose.getTranslation(), targetDist)) 
    {
      currentWaypoint = Math.min(++currentWaypoint, waypoints.size());
      onPath = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Conversions.atPose(robotPose, waypoints.get(waypoints.size()-1));
  }
}
