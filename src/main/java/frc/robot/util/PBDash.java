package frc.robot.util;

import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;

/** 
 * Simplified interface for most dashboard/network-table interactions 
 * @author 5985
 */
public class PBDash 
{
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable(IDConstants.dashTableName);

  public static final Field2d FIELD = new Field2d();
  static { putSendable("Field", FIELD); }

  // Auto-builder strings
  public static final Key<String>  AUTO_STRING      = new Key<>("Auto String", "");
  public static final Key<String>  AUTO_ERRS        = new Key<>("AUTO STRING ERRORS", "");

  public static final Key<Boolean>  LAUNCHPAD_GOOD  = new Key<>("Launchpad Good", true);
  
  // System switches and buttons
  public static final Key<Boolean> IO_LL            = new Key<>("Use Limelight", true);
  public static final Key<Boolean> IO_FENCE         = new Key<>("Enable Fencing", true);
  public static final Key<Boolean> IO_AUTO_AIM      = new Key<>("Auto Aim", true);
  public static final Key<Boolean> IO_AUTO_PASS     = new Key<>("Auto Pass", true);
  public static final Key<Boolean> IO_AUTO_REV      = new Key<>("Auto Rev", true);

  // State displays
  public static final Key<String>  STATE_DRIVE      = new Key<>("Drive State", "Disabled");
  public static final Key<Boolean> STATE_NUDGING    = new Key<>("Nudging Active", true);

  // Request queues
  public static final Key<Double>  RUMBLE_DRIVER    = new Key<>("Driver Rumble", Constants.RumblerConstants.driverDefault);
  public static final Key<Double>  RUMBLE_OPERATOR  = new Key<>("Operator Rumble", Constants.RumblerConstants.operatorDefault);

  // Testing values
  public static final Key<Double>  TEST_FLYSPEED    = new Key<>("Test Flyspeed", 0.0);
  public static final Key<Double>  TEST_AZIMUTH     = new Key<>("Test Azimuth", 0.0);
  public static final Key<Double>  TEST_ALTITUDE    = new Key<>("Test Altitude", 0.0);

  public static final Key<Double>  TEST_INTAKE_SPEED= new Key<>("Test Intake Speed", Constants.HopperConstants.IntakeConstants.intakeSpeed);

  public static void putFieldObject(String name, Pose2d pose)
    {FIELD.getObject(name).setPose(pose);}

  public static void putFieldObject(String name, Pose2d... poses)
    {FIELD.getObject(name).setPoses(poses);}

  public static void putFieldObject(String name, Translation2d point)
    {FIELD.getObject(name).setPose(new Pose2d(point, Rotation2d.kZero));}

  public static void putFieldPath(String name, Pose2d start, Pose2d end)
  {
    // Elastic only displays a trajectory for objects with 8+ poses, so we generate a bunch of intermediate poses to force it

    double length = start.getTranslation().getDistance(end.getTranslation());

    double sectionLength = length / 8;
    var poses = IntStream
      .range(0, 9)
      .boxed()
      .map(section -> start.interpolate(end, sectionLength * section))
      .collect(Collectors.toList());

    FIELD.getObject(name).setPoses(poses);
  }

  /**
   * Publishes a Sendable to the table {@value IDConstants#dashTableName} <p>
   * NOTE: Only publish each Sendable once, they will automatically be periodically updated 
   * 
   * @param name name to use for the published value
   * @param value value to publish
   */
  public static void putSendable(String name, Sendable data) 
  {
    NetworkTable dataTable = table.getSubTable(name);

    SendableBuilderImpl builder = new SendableBuilderImpl();
    builder.setTable(dataTable);
    SendableRegistry.publish(data, builder);
    builder.startListeners();

    dataTable.getEntry(".name").setString(name);
  }

  /**
   * Publishes an int to the table {@value IDConstants#dashTableName}
   * 
   * @param name name to use for the published value
   * @param value value to publish
   */
  public static void putInt(String name, int value)
    {entry(name).setInteger(Long.valueOf(value));}

  /**
   * Publishes a double to the table {@value IDConstants#dashTableName}
   * 
   * @param name name to use for the published value
   * @param value value to publish
   */
  public static void putDouble(String name, double value)
    {entry(name).setDouble(value);}

  /**
   * Publishes a boolean to the table {@value IDConstants#dashTableName}
   * 
   * @param name name to use for the published value
   * @param value value to publish
   */
  public static void putBool(String name, Boolean value)
    {entry(name).setBoolean(value);}

  /**
   * Publishes a String to the table {@value IDConstants#dashTableName}
   * 
   * @param name name to use for the published value
   * @param value value to publish
   */
  public static void putString(String name, String value)
    {entry(name).setString(value);}

  /**
   * Gets an int from the table {@value IDConstants#dashTableName} <p>
   * If a value with the given name is not present in the table, {@code 0} will be pushed to network and returned
   * 
   * @param name name of the value to get
   */
  public static int getInt(String name)
  {
    if(!entry(name).exists()) putInt(name, 0);
    return (int)entry(name).getInteger(0);
  }

  /**
   * Gets a double from the table {@value IDConstants#dashTableName} <p>
   * If a value with the given name is not present in the table, {@code 0.0} will be pushed to network and returned
   * 
   * @param name name of the value to get
   */
  public static double getDouble(String name)
  {
    if(!entry(name).exists()) putDouble(name, 0);
    return entry(name).getDouble(0);
  }

  /**
   * Gets a boolean from the table {@value IDConstants#dashTableName} <p>
   * If a value with the given name is not present in the table, {@code false} will be pushed to network and returned
   * 
   * @param name name of the value to get
   */
  public static boolean getBool(String name)
  {
    if(!entry(name).exists()) putBool(name, false);
    return entry(name).getBoolean(false);
  }

  /**
   * Gets a String from the table {@value IDConstants#dashTableName} <p>
   * If a value with the given name is not present in the table, an empty string will be pushed to network and returned
   * 
   * @param name name of the value to get
   */
  public static String getString(String name)
  {
    if(!entry(name).exists()) putString(name, "");
    return entry(name).getString("");
  }

  /** Internal helper to make some lines shorter */
  private static final NetworkTableEntry entry(String name)
    {return table.getEntry(name);}

  /** 
   * A generic class encapslating a NetworkTable entry, adding additional safety and providing methods for ease of interaction. <p>
   * Primarily intended to be stored as a constant
   */
  public static class Key<T>
  {
    private T defaultVal;
    private GenericEntry ntEntry;
    private T lastVal;

    /**
     * Construct a new Key
     * 
     * @param label name to give the underlying NT entry
     * @param defaultVal initialisation value. pushed to NT immediately 
     */
    public Key(String label, T defaultVal)
    {
      this.defaultVal = defaultVal;
      ntEntry = table.getTopic(label).getGenericEntry();
      init();
    }

    /** @return current value of the entry */
    @SuppressWarnings("unchecked")
    public T get()
    {
      lastVal = (T)ntEntry.get().getValue();
      return lastVal;
    }

    /** @param value value to send to network */
    public void put(T value)
      {ntEntry.setValue(value);}

    /** Sends the default value to network */
    public void init()
      {put(defaultVal);}

    /** @return default value */
    public T defaultVal()
      {return defaultVal;}

    /** @return {@code true} if the entry's value has changed since the last call to this or to {@link Key#get get()} */
    @SuppressWarnings("unchecked")
    public boolean hasChanged()
    {
      T newVal = (T)ntEntry.get().getValue();
      boolean result = (lastVal == null) || (!lastVal.equals(newVal));
      lastVal = newVal;
      return result;
    }

    /**
     * If the entry has changed from the default value, resets the value and returns true. <p>
     * Primarily for use with a Key<Boolean> with the default as false, so that it acts as a self-resetting button <p>
     * If you use this for anything other than boolean... why?
     * 
     * @return whether the entry has changed from it's default value
     */
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

    /** @return Trigger monitoring if the value has changed */
    public Trigger asTrigger()
      {return new Trigger(() -> hasChanged());}

    /** @return Trigger monitoring if the value has changed then resetting the value */
    public Trigger asButton()
      {return new Trigger(() -> button());}

    /** @return Trigger of value being `true` */
    public Trigger asSwitch()
      {return new Trigger(() -> get().equals(true));}

    /** Closes the underlying entry. <p> ATTEMPTING TO USE A KEY AFTER CLOSING IT WILL CAUSE ERRORS */
    public void close()
      {ntEntry.close();}
  }

  /**
   * Initialises a dashboard display showing the angle and speed of each of the drivebase's swerve modules
   * 
   * @param swerveStateSup supplier to get the module states via
   */
  public static void initSwerveDisplay(Supplier<SwerveDriveState> swerveStateSup)
  {
    putSendable
    (
      "Swerve Drive", 
      new Sendable() 
      {
        @Override
        public void initSendable(SendableBuilder builder) 
        {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty("Front Left Angle", () -> swerveStateSup.get().ModuleStates[0].angle.getRadians(), null);
          builder.addDoubleProperty("Front Left Velocity", () -> swerveStateSup.get().ModuleStates[0].speedMetersPerSecond, null);

          builder.addDoubleProperty("Front Right Angle", () -> swerveStateSup.get().ModuleStates[1].angle.getRadians(), null);
          builder.addDoubleProperty("Front Right Velocity", () -> swerveStateSup.get().ModuleStates[1].speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Left Angle", () -> swerveStateSup.get().ModuleStates[2].angle.getRadians(), null);
          builder.addDoubleProperty("Back Left Velocity", () -> swerveStateSup.get().ModuleStates[2].speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Right Angle", () -> swerveStateSup.get().ModuleStates[3].angle.getRadians(), null);
          builder.addDoubleProperty("Back Right Velocity", () -> swerveStateSup.get().ModuleStates[3].speedMetersPerSecond, null);

          builder.addDoubleProperty("Robot Angle", () -> swerveStateSup.get().Pose.getRotation().getRadians(), null);
        }
      }
    );
  }
}
