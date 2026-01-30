// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.Constants.HopperConstants.*;
import static frc.robot.constants.Constants.HopperConstants.ExtensionConstants.extensionJostleDelay;

public class Hopper extends SubsystemBase 
{
  private BinaryMotor spindexer;
  private BinaryMotor intake;
  private LinearExtension extension;
  
  /** Creates a new Hopper. */
  public Hopper(int spindexerCAN, int intakeCAN, int extensionCAN, int extensionLimitCAN) 
  { 
    /** Creates new defintions to be used in the commands*/
    spindexer = new BinaryMotor(spindexerSpeed, spindexerCAN);
    intake = new BinaryMotor(intakeSpeed, intakeCAN);
    extension = new LinearExtension(extensionCAN, extensionLimitCAN, 0, ExtensionConstants.maxRotations, ExtensionConstants.config);
  }
  
  /**@return Command the intake of the fuel
   * Starts intaking fuel
  */
  public Command runIntakeCommand()
  {return intake.startCommand();}

  //Stops intaking fuel
  public Command stopIntakeCommand()
  {return intake.stopCommand();}
  
  /**@return Command for the spindexer*/
  //Starts running the spindexer
  public Command runSpindexerCommand()
  {return spindexer.startCommand();}

  // Stops the spindexer
  public Command stopSpindexerCommand()
  {return spindexer.stopCommand();}

  //Rapidly pulses the spindexer in order to remove any jammed fuel
  public Command pulseSpindexerCommand()
  {
    return 
    Commands.sequence
    (
      runSpindexerCommand(),
      //Waits for 0.25 seconds
      Commands.waitSeconds(spindexerPulseDelay),
      stopSpindexerCommand(),
      Commands.waitSeconds(spindexerPulseDelay)
    )
    .repeatedly();
  }

  /**@return Command for all the extensions */
  //Retracts the extension
  public Command retractCommand()
  {return extension.setTargetCommand(0);}

  //Extends the extension until it's at it's max rotations
  public Command extendCommand()
  {return extension.setTargetCommand(ExtensionConstants.maxRotations);}

  // Creates a command that jostles the extension to remove jammed fuel
  public Command extensionJostleCommand()
  {
    return 
    Commands.sequence
    (
      extendCommand(), 
      //Waits for 0.25 seconds
      Commands.waitSeconds(extensionJostleDelay),
      retractCommand(),
      Commands.waitSeconds(extensionJostleDelay)
    )
    .repeatedly();
  }
  @Override
  public void periodic() {}
}
