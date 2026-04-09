package frc.robot.constants;

/** 
 * CAN IDs, PWM ports, IO ports, device names, etc.
 * @author 5985
 */
public final class IDConstants 
{   
  public static final int pdhCAN = 0;

  /* Drive */
  /* ----- */
  public static final int foreStbdDriveMotorCAN = 1;
  public static final int foreStbdAngleMotorCAN = 2;
  public static final int foreStbdCANcoderCAN   = 3;

  public static final int forePortDriveMotorCAN = 4;
  public static final int forePortAngleMotorCAN = 5;
  public static final int forePortCANcoderCAN   = 6;

  public static final int aftPortDriveMotorCAN = 7;
  public static final int aftPortAngleMotorCAN = 8;
  public static final int aftPortCANcoderCAN   = 9;

  public static final int aftStbdDriveMotorCAN = 10;
  public static final int aftStbdAngleMotorCAN = 11;
  public static final int aftStbdCANcoderCAN   = 12;

  public static final int pigeonCAN = 13;

  /* Mechanism */
  /* --------- */

  public static final int LEDPWDPort = 2;

  public record ShooterIDs(int flywheelLeadCAN, int flywheelFollowCAN, int azimuthCAN, int indexerCAN, int azimuthAIO, int altitudePWM, int altitudeAIO, String ntID){}

  /* Port Turret, CAN [16..19], PWM [0], AIO [0] */
  public static final ShooterIDs portShooterIDs = new ShooterIDs
  (
    16,
    17,
    18,
    19,
    0,
    0,
    2,
    "Port"
  );

  /* Stbd Turret, CAN [20..23], PWM [1], AIO [1] */
  public static final ShooterIDs stbdShooterIDs = new ShooterIDs
  (
    20,
    21,
    22,
    23,
    1,
    1,
    3,
    "Stbd"
  );

  /* Intake, CAN [28..30] */
  public static final int extensionEncoderCAN = 28;
  public static final int intakeCAN = 29;
  public static final int extensionCAN = 30;

  /* Climber, CAN [32], DIO [0..1] */
  public static final int climberCAN = 32;
  public static final int climberLimitDIO = 0;
  public static final int climberPostDIO = 1;
  
  /* Network device names */
  /* -------------------- */
  public static final String portLimelightName = "PhotonPort";
  public static final String stbdLimelightName = "PhotonStbd";

  public static final String dashTableName = "PBDash";

  /* Controllers */
  /* ----------- */
  public static final int driverPort = 0;
  public static final int debugPort = 1;
  public static final int switchboardPort = 2;

  /* Switchboard Switches */
  public static final int autoAimSwitchID = 4;
  public static final int shootHubSwitchID = 6;
  public static final int shootPassSwitchID = 5;
  public static final int fencingSwitchID = 7;
  public static final int visionSwitchID = 8;
  public static final int climbButtonID = 9;
}
