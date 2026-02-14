package frc.robot.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;

/** 
 * Simplified interface for most dashboard/network-table interactions 
 * @author 5985
 */
public class PBDash 
{
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable(IDConstants.dashTableName);

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

  public static final Key<Double> CAN_LOAD              = new Key<>("CAN-bus Load", 0.0);

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
    SmartDashboard.putData
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
