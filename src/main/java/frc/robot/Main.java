// Copyright (c) FIRST and other WPILib contributors. // Copyright info (did I just comment a comment???)
// Open Source Software; you can modify and/or share it under the terms of // Licensing (I did)
// the WPILib BSD license file in the root directory of this project. // Licensing, cont. (Help)

package frc.robot; // The folder this is in (packages are kinda weird tbh)

import edu.wpi.first.wpilibj.RobotBase; // Importing RobotBase so we can start the robot

public final class Main { // Defines the Main class
  private Main() {} // Constructor that does nothing
  // Standard whitespace for organisation- oops it's now a comment
  public static void main(String... args) { // Defines the (overly verbose, thx Java <3) main method
    RobotBase.startRobot(Robot::new); // Starts the robot using a new Robot (this comment makes no sense)
  } // Terminates the main method
} // Terminates the Main class
// Have a nice day :)