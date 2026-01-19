// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.SD;

import static frc.robot.constants.Constants.Shooter.*;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  public enum Target 
  {
    Manual, 
    Point, 
    Hub;

    public double hoodAngle = 0;
    public double heading = 0;
    public Translation2d point = Translation2d.kZero;

    public Target set(Translation2d point) {
      this.point = point;
      return this;
    }

    public Target set(double heading, double hoodAngle) {
      this.heading = heading;
      this.hoodAngle = hoodAngle;
      return this;
    } 
  }

  private Target target = Target.Hub;

  private final Flywheels flywheels; 
  private final Turret turret;
  private final Hood hood;

  private final Supplier<Pose2d> robotPoseSup;

  /** Creates a new shooter. */
  public Shooter
  (
    Supplier<Pose2d> robotPoseSup,
    int shooterMainID, 
    int shooterAuxID, 
    int turretID, 
    int hoodID
  ) 
  {
    this.robotPoseSup = robotPoseSup;

    flywheels = new Flywheels(shooterMainID, shooterAuxID);
    turret = new Turret(turretID);
    hood = new Hood(hoodID, Transform2d.kZero);
  }

  public void setTarget(Target target)
  {
    this.target = target;
  }

  @Override
  public void periodic()
  {
    turret.update(robotPoseSup.get(), target);
    hood.update(robotPoseSup.get(), target);
  }
}
