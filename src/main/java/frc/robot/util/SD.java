// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Simplified interface for most SmartDashboard interactions */
public class SD 
{
  public static final StringKey  AUTO_STRING      = new StringKey("Auto String", "");

  public static final DoubleKey  LL_EXPOSURE      = new DoubleKey("Exposure Setting", 0);
  public static final BooleanKey LL_EXPOSURE_UP   = new BooleanKey("Increase Exposure", false);
  public static final BooleanKey LL_EXPOSURE_DOWN = new BooleanKey("Decrease Exposure", false);
  public static final BooleanKey LL_TOGGLE        = new BooleanKey("Use Limelight", true);

  public static final BooleanKey FENCE_TOGGLE     = new BooleanKey("Enable Fencing", true);

  public static final StringKey  STATE_HEADING    = new StringKey("Heading State", "");
  public static final StringKey  STATE_DRIVE      = new StringKey("Drive State", "Disabled");

  public static final DoubleKey  RUMBLE_DRIVER    = new DoubleKey("Driver Rumble", Constants.RumblerConstants.driverDefault);
  public static final DoubleKey  RUMBLE_OPERATOR  = new DoubleKey("Operator Rumble", Constants.RumblerConstants.operatorDefault);

  static
  {
    try 
    {
      for (var field : SD.class.getFields())
      {
        if (field.get(null) instanceof Key key) key.init();
      }
    } catch (Exception e) {/* This will verifiably never happen */}
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

  private interface Key<T> extends Supplier<T>, Consumer<T>
  {
    public default void accept(T value) {put(value);}

    public T get();

    public void put(T value);

    public void init();
  }

  public record BooleanKey (String label, boolean defaultValue) implements Key<Boolean>
  {
    public Boolean get() {return SmartDashboard.getBoolean(label, defaultValue);}

    public boolean button() 
    {
      if (get()) 
      {
        put(false); 
        return true;
      } 
      else 
        {return false;}
    }

    public void init() {SmartDashboard.putBoolean(label, defaultValue);}

    public void put(Boolean value) {SmartDashboard.putBoolean(label, value);}
  }

  public record DoubleKey (String label, double defaultValue) implements Key<Double>
  {
    public Double get() {return SmartDashboard.getNumber(label, defaultValue);}

    public void init() {SmartDashboard.putNumber(label, defaultValue);}

    public void put(Double value) {SmartDashboard.putNumber(label, value);}
  }

  public record StringKey (String label, String defaultValue) implements Key<String>
  {
    public String get() {return SmartDashboard.getString(label, defaultValue);}

    public void init() {SmartDashboard.putString(label, defaultValue);}

    public void put(String value) {SmartDashboard.putString(label, value);}
  }
}
