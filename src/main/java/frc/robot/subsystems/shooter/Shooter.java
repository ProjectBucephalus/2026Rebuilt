// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.SD;

import static frc.robot.constants.Constants.Shooter.*;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private final Flywheels flywheels; 
  private final Turret turret;
  private final Hood hood;

  private final Supplier<Rotation2d> robotRotationSup;
  private final Supplier<Translation2d> robotPositionSup;

  /** Creates a new shooter. */
  public Shooter
  (
    Supplier<Translation2d> translationSup, 
    Supplier<Rotation2d> rotationSup, 
    int shooterMainID, 
    int shooterAuxID, 
    int turretID, 
    int hoodID
  ) 
  {
    robotRotationSup = rotationSup;
    robotPositionSup = translationSup;

    flywheels = new Flywheels(shooterMainID, shooterAuxID);
    turret = new Turret(turretID);
    hood = new Hood(hoodID);
  }

  @Override
  public void periodic()
  {
    turret.update(robotPositionSup.get(), robotRotationSup.get());
  }
}
