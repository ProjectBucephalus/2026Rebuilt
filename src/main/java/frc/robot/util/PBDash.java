// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Simplified interface for most SmartDashboard interactions */
public class PBDash 
{
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("PBDash");

  public static final Key<String>  AUTO_STRING          = new Key<>("Auto String", "");

  public static final Key<Integer> LL_EXPOSURE          = new Key<>("Exposure Setting", 0);
  public static final Key<Boolean> LL_EXPOSURE_UP       = new Key<>("Increase Exposure", false);
  public static final Key<Boolean> LL_EXPOSURE_DOWN     = new Key<>("Decrease Exposure", false);
  public static final Key<Boolean> LL_TOGGLE            = new Key<>("Use Limelight", true);

  public static final Key<Boolean> FENCE_TOGGLE         = new Key<>("Enable Fencing", true);

  public static final Key<String>  STATE_HEADING        = new Key<>("Heading State", "");
  public static final Key<String>  STATE_DRIVE          = new Key<>("Drive State", "Disabled");

  public static final Key<Double>  RUMBLE_DRIVER        = new Key<>("Driver Rumble", Constants.RumblerConstants.driverDefault);
  public static final Key<Double>  RUMBLE_OPERATOR      = new Key<>("Operator Rumble", Constants.RumblerConstants.operatorDefault);

  public static final Key<Double>  BOTTOM_SHOOTER_SPEED = new Key<>("Bottom Shooter Speed", 0.0);
  public static final Key<Double>  TOP_SHOOTER_SPEED    = new Key<>("Top Shooter Speed", 0.0);

  public static <T> void put(String name, T value)
  {
    var publisher = table.getTopic(name).genericPublish(value.getClass().getSimpleName());
      {publisher.setValue(value);}
  }

  public static <T> T get(String name)
  {
    var subscriber = table.getTopic(name).genericSubscribe();
      {return (T)subscriber.get().getValue();}
  }

  public static class Key<T>
  {
    private T defaultVal;
    private GenericEntry ntEntry;

    public Key(String label, T defaultVal)
    {
      this.defaultVal = defaultVal;
      ntEntry = table.getTopic(label).getGenericEntry();
      init();
    }

    @SuppressWarnings("unchecked")
    public T get()
      {return (T)ntEntry.get().getValue();}

    public void put(T value)
      {ntEntry.setValue(value);}

    public void init()
      {put(defaultVal);}

    public T defaultVal()
      {return defaultVal;}

    public boolean button()
    {
      if (get() != defaultVal)
      {
        init();
        return true;
      } 
      else 
        return false;
    }

    public void close()
      {ntEntry.close();}
  }

  public static void initSwerveDisplay(CommandSwerveDrivetrain s_Swerve)
  {
    SmartDashboard.putData
    (
      "Swerve Drive", 
      new Sendable() 
      {
        @Override
        public void initSendable(SendableBuilder builder) 
        {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty("Front Left Angle", () -> s_Swerve.getModule(0).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Front Left Velocity", () -> s_Swerve.getModule(0).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Front Right Angle", () -> s_Swerve.getModule(1).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Front Right Velocity", () -> s_Swerve.getModule(1).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Left Angle", () -> s_Swerve.getModule(2).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Back Left Velocity", () -> s_Swerve.getModule(2).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Right Angle", () -> s_Swerve.getModule(3).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Back Right Velocity", () -> s_Swerve.getModule(3).getCurrentState().speedMetersPerSecond, null);

          //!TODO FIX THIS
          //DoubleProperty("Robot Angle", () -> RobotContainer.swerveState.Pose.getRotation().getRadians(), null);
        }
      }
    );
  }
}
