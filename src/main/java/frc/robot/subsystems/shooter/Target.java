package frc.robot.subsystems.shooter;

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
  public double altitude; 
  public double azimuth; 
  public Translation2d point;


  // creates a function accesible to everything to store where a target is
  public Target(TargetState state, double altitude, double azimuth, Translation2d point) 
  {
    this.state = state;
    this.altitude = altitude;
    this.azimuth = azimuth;
    this.point = point;
  }

  public Target(TargetState state) 
    {this(state, 0, 0, Translation2d.kZero);}
}