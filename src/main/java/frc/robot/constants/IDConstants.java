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

  public record ShooterIDs(int flywheelLeadCAN, int flywheelFollowCAN, int azimuthCAN, int azimuthAIO, int altitudePWM, int altitudeAIO, String ntID){}

  /* Port Turret, [16..18], PWM/AIO [0] */
  public static final ShooterIDs portShooterIDs = new ShooterIDs
  (
    16,
    17,
    18,
    0,
    1,
    1,
    "Port"
  );

  /* Stbd Turret, [20..22], PWM/AIO [1] */
  public static final ShooterIDs stbdShooterIDs = new ShooterIDs
  (
    20,
    21,
    22,
    2,
    3,
    3,
    "Stbd"
  );

  /* Feeder, [24..25] */
  public static final int portFeederCan = 24;
  public static final int stbdFeederCan = 25;

  /* Processor, [28..30] */
  public static final int spindexerCAN = 28;
  public static final int intakeCAN = 29;
  public static final int extensionCAN = 30;

  /* Climber, [32] */
  public static final int climberCAN = 32;

  /* Sensors */
  /* ------- */
  public static final int extensionLimitDIO = 0;
  public static final int climberLimitDIO = 1;
  
  /* Network device names */
  /* -------------------- */
  public static final String portLimelightName = "PhotonPort";
  public static final String stbdLimelightName = "PhotonStbd";

  public static final String dashTableName = "PBDash";
}
