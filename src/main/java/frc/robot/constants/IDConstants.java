// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** 
 * CAN IDs, PWM ports, IO ports, device names, etc.
 * @author 5985
 */
public final class IDConstants 
{   
  public static final int pdhCAN = 0;

  /* Drive */
  /* _____ */
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
  /* _________ */

  /* Port Turret, [16..18], PWM [1] */
  public static final int portFlyLeaderCAN = 16;
  public static final int portFlyFollowerCAN = 17;
  public static final int portTurretCAN = 18;
  public static final int portPotIO = 1;
  public static final int portHoodPWM = 1;

  /* Stbd Turret, [20..22], PWM [2] */
  public static final int stbdFlyLeaderCAN = 20;
  public static final int stbdFlyFollowerCAN = 21;
  public static final int stbdTurretCAN = 22;
  public static final int stbdPotIO = 2;
  public static final int stbdHoodPWM = 2;

  /* Feeder, [24] */
  public static final int feederCAN = 24;

  /* Processor, [28..30] */
  public static final int spindexerCAN = 28;
  public static final int intakeCAN = 29;
  public static final int extensionCAN = 30;

  /* Climber, [32] */
  public static final int climberCAN = 32;

  /* Limelights */
  /* __________ */
  public static final String portLimelightName = "PhotonPort";
  public static final String stbdLimelightName = "PhotonStbd";

  /* Sensors */
  /* _______ */
  public static final int extensionLimitDIO = 1;
  public static final int climberLimitDIO = 2;

  /* NT Table Name */
  public static final String dashTableName = "PBDash";
}
