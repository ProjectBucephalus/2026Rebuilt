package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Target 
{
  // creates states that are used in Hood
  public enum TargetState 
  {
    Manual, 
    Point, 
    Hub;
  }

  public TargetState state; 
  public Rotation2d altitude; 
  public Rotation2d azimuth; 
  public Translation2d point;

  // creates a function accesible to everything to store where a target is
  public Target(TargetState state, Rotation2d altitude, Rotation2d azimuth, Translation2d point) 
  {
    this.state = state;
    this.altitude = altitude;
    this.azimuth = azimuth;
    this.point = point;
  }

  public Target(TargetState state) 
    {this(state, Rotation2d.kZero, Rotation2d.kZero, Translation2d.kZero);}
}