package frc.robot.util;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
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
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Constants.*;

/** 
 * Simplified interface for most dashboard/network-table interactions 
 * @author 5985
 */
public class PBDash 
{
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable(IDConstants.dashTableName);
  private static final Map<String, Sendable> tablesToData = new HashMap<>();

  // System states
  public static final Key<String> DEVICE_ERRORS = new Key<>("Device Errors", "");
  public static final Key<String> CLIMBER_STATE = new Key<>("Climber State", "Home");
  public static final Key<String> EXTENSION_STATE = new Key<>("Extension State", "Stowed");

  // Auto-builder
  public static final Key<String>  AUTO_STRING      = new Key<>("Auto String", "");
  public static final Key<String>  AUTO_ERRS        = new Key<>("Auto String Errors", "");
  public static final SendableChooser<String> AUTO_PRESETS = new SendableChooser<>();
  static
  {
    AUTO_PRESETS.setDefaultOption(ControlConstants.autoPresets[0].getFirst(), ControlConstants.autoPresets[0].getSecond());
    for (int i = 1; i < ControlConstants.autoPresets.length; i++)
    {
      var preset = ControlConstants.autoPresets[i];
      AUTO_PRESETS.addOption(preset.getFirst(), preset.getSecond());
    }
    AUTO_PRESETS.onChange(AUTO_STRING::put);
    putSendable("Auto Presets", AUTO_PRESETS);
  }

  // System switches and buttons
  public static final Key<Boolean> IO_LL            = new Key<>("Use Limelight", true);
  public static final Key<Boolean> IO_FENCE         = new Key<>("Enable Fencing", true);
  public static final Key<Boolean> IO_SHOOT_HUB     = new Key<>("Auto Shoot Hub", true);
  public static final Key<Boolean> IO_SHOOT_PASS    = new Key<>("Auto Shoot Pass", false);
  public static final Key<Boolean> IO_POWER_DRIVE   = new Key<>("Power Save Drive", false);
  public static final Key<Boolean> IO_POWER_SHOOT   = new Key<>("Power Save Shooter", false);
  public static final Key<Boolean> IO_CLIMB_WIGGLE  = new Key<>("Climb Wiggle", true);

  // State feedback
  public static final Key<String>   STATE_DRIVE     = new Key<>("Drive State", "");
  public static final Key<String[]> STATE_LED_DRIVE = new Key<>("Drive State LED", new String[]{});
  public static final Key<String[]> STATE_LED_PORT  = new Key<>("Shooter State LED Port", new String[]{});
  public static final Key<String[]> STATE_LED_STBD  = new Key<>("Shooter State LED Stbd", new String[]{});
  
  // Rumble strengths
  public static final Key<Double>  RUMBLE_DRIVER    = new Key<>("Driver Rumble", RumblerConstants.driverDefault);
  public static final Key<Double>  RUMBLE_OPERATOR  = new Key<>("Operator Rumble", RumblerConstants.operatorDefault);
  
  // Testing values
  public static final Key<Double>  TEST_FLYSPEED    = new Key<>("Test Flyspeed", 0.0);
  public static final Key<Double>  TEST_AZIMUTH     = new Key<>("Test Azimuth", 0.0);
  public static final Key<Double>  TEST_ALTITUDE    = new Key<>("Test Altitude", 0.0);

  // Tuning values
  public static final Key<Double>  TUNE_MECH_LAG    = new Key<>("Mechanism Lag", ShooterConstants.mechanismLag);
  public static final Key<Double>  TUNE_LEAD_FACTOR = new Key<>("Acceleration Lead Factor", ShooterConstants.accelLeadFactor);
  public static final Key<Double>  TUNE_ACCEL_LIMIT = new Key<>("Acceleration Limit", ShooterConstants.leadingAccelLimit);
  public static final Key<Double>  TUNE_JERK_LIMIT  = new Key<>("Jerk Limit", ShooterConstants.leadingJerkLimit);
  public static final Key<Double>  TUNE_CLIMB_BL    = new Key<>("Cl. Off. Blue Left", FieldConstants.FieldTuning.climbOffsetBlueLeft);
  public static final Key<Double>  TUNE_CLIMB_RL    = new Key<>("Cl. Off. Red Left", FieldConstants.FieldTuning.climbOffsetRedLeft);
  public static final Key<Double>  TUNE_CLIMB_BR    = new Key<>("Cl. Off. Blue Right", FieldConstants.FieldTuning.climbOffsetBlueRight);
  public static final Key<Double>  TUNE_CLIMB_RR    = new Key<>("Cl. Off. Red Right", FieldConstants.FieldTuning.climbOffsetRedRight);
  
  // Manual speed adjustment
  public static final Key<Double>  IO_INTAKE_SPEED  = new Key<>("Max Intake Speed", IntakeConstants.RollerConstants.intakeMaxSpeed);
  public static final Key<Double>  IO_MAX_THROTTLE  = new Key<>("Max Throttle", ControlConstants.maxThrottle);
  public static final Key<Double>  IO_MIN_THROTTLE  = new Key<>("Min Throttle", ControlConstants.minThrottle);

  // Robot pose
  public static final Key<String>  POSE             = new Key<>("Robot Pose", "");
  public static final Key<String>  POSE_FINE        = new Key<>("Robot Pose mm", "");
  public static final Field2d      FIELD            = new Field2d();
  static { putSendable("Field", FIELD); }

  public static void putFieldObject(String name, Translation2d point)
    {FIELD.getObject(name).setPose(new Pose2d(point, Rotation2d.kZero));}

  public static void putFieldObject(String name, Pose2d... poses)
    {FIELD.getObject(name).setPoses(poses);}

  public static void addToFieldObject(String name, Pose2d... newPoses)
  {
    var object = FIELD.getObject(name);
    var poses = object.getPoses();
    // Elastic only displays a trajectory for objects with 8+ poses, so we add the first pose a bunch of times to force it
    for (int i = 0; i < 9 - poses.size(); i++) poses.add(newPoses[0]);
    poses.addAll(List.of(newPoses));
    object.setPoses(poses);
  }

  public static void putFieldPath(String name, Pose2d start, Pose2d end)
  {
    // Elastic only displays a trajectory for objects with 8+ poses, so we generate a bunch of intermediate poses to force it

    double length = start.getTranslation().getDistance(end.getTranslation());

    double sectionLength = length / 8;
    var poses = IntStream
      .range(0, 9)
      .boxed()
      .map(section -> start.interpolate(end, sectionLength * section))
      .toList();

    FIELD.getObject(name).setPoses(poses);
  }

  public static void trailFieldObject(String name, Pose2d newPose)
  {
    var object = FIELD.getObject(name);
    var poses = object.getPoses();
    // Elastic only displays a trajectory for objects with 8+ poses, so we add the first pose a bunch of times to force it
    for (int i = 0; i < (50 - poses.size()); i++) poses.add(newPose);
    poses.add(newPose);
    poses.remove(0);
    object.setPoses(poses);
  }

  public static void removeFieldObject(String name)
    {FIELD.getObject(name).setPoses();}

  /**
   * Publishes a Sendable to the table {@value IDConstants#dashTableName} <p>
   * NOTE: Only publish each Sendable once, they will automatically be periodically updated 
   * 
   * @param name name to use for the published value
   * @param value value to publish
   */
  public static void putSendable(String name, Sendable data) 
  {
    Sendable sddata = tablesToData.get(name);
    if (sddata == null || sddata != data) 
    {
      tablesToData.put(name, data);
      NetworkTable dataTable = table.getSubTable(name);
      SendableBuilderImpl builder = new SendableBuilderImpl();
      builder.setTable(dataTable);
      SendableRegistry.publish(data, builder);
      builder.startListeners();
      dataTable.getEntry(".name").setString(name);
    }
  }

  public static void updateSendables()
  {
    for (Sendable data : tablesToData.values()) 
      SendableRegistry.update(data);
  }

  /**
   * Publishes an int to the table {@value IDConstants#dashTableName}
   * 
   * @param name name to use for the published value
   * @param value value to publish
   */
  public static void putInt(String name, int value)
    {entry(name).setInteger(value);}

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
    private final T defaultVal;
    private final GenericEntry ntEntry;

    private final Trigger mainTrigger = new Trigger(() -> get().equals(true));
    private final Trigger btnTrigger = new Trigger(() -> button());
    private final Trigger pulseTrigger = new Trigger(() -> hasChanged());

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
      {return (T)ntEntry.get().getValue();}

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
    public Trigger asPulse()
      {return pulseTrigger;}

    /** @return Trigger monitoring if the value has changed then resetting the value */
    public Trigger asButton()
      {return btnTrigger;}

    /** @return Trigger of value being `true` */
    public Trigger asTrigger()
      {return mainTrigger;}
    
    /**
     * Appends the provided text to the current value of the key, or does nothing if this is not a Key<String>
     * @param text The text to append
     */
    @SuppressWarnings("unchecked")
    public void append(String text)
    {
      // The cast from String to T will only ever happen is T is already String
      if (get() instanceof String str)
        put((T)(str + text));
    }

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
      builder -> 
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
    );
  }
}
