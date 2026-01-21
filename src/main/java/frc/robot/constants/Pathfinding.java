// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Constraints and waypoint sequences for use with PathFollowDrive commands */
public abstract class Pathfinding 
{
    public static final Rotation2d testPathRotation = Rotation2d.k180deg;
    public static final Translation2d[] testPath = new Translation2d[] 
    {
        new Translation2d(15, 2),
        new Translation2d(11, 2),
        new Translation2d(11, 6)
    };
}
