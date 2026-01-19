// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

public final class IDConstants 
{   
  public static final int pdhID = 0;

  /* Drive */
  /* _____ */
  public static final int foreStbdDriveMotorID = 1;
  public static final int foreStbdAngleMotorID = 2;
  public static final int foreStbdCANcoderID   = 3;

  public static final int forePortDriveMotorID = 4;
  public static final int forePortAngleMotorID = 5;
  public static final int forePortCANcoderID   = 6;

  public static final int aftPortDriveMotorID = 7;
  public static final int aftPortAngleMotorID = 8;
  public static final int aftPortCANcoderID   = 9;

  public static final int aftStbdDriveMotorID = 10;
  public static final int aftStbdAngleMotorID = 11;
  public static final int aftStbdCANcoderID   = 12;

  public static final int pigeonID = 53; // Pigeon ID hardset to 53, we can't change it

  /* Mechanism */
  /* _________ */
  public static final int shooterMainID = 22;
  public static final int shooterAuxID = 23;
  
  public static final int turretID = 0;

  /* Limelights */
  /* __________ */
  public static final String foreLimelightName = "LemonLightPort";
  public static final String aftLimelightName = "LemonLightStbd";
}
