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
    spindexer = new BinaryMotor(spindexerSpeed, spindexerCAN);
    intake = new BinaryMotor(intakeSpeed, intakeCAN);
    extension = new LinearExtension(extensionCAN, extensionLimitCAN, 0, ExtensionConstants.maxRotations, ExtensionConstants.config);
  }
  
  /**@return Command to intake*/
  public Command runIntakeCommand()
  {return intake.startCommand();}

  public Command stopIntakeCommand()
  {return intake.stopCommand();}
  
  /** @return Command to start spindexe */
  public Command runSpindexerCommand()
  {return spindexer.startCommand();}

  public Command stopSpindexerCommand()
  {return spindexer.stopCommand();}

  public Command pulseSpindexerCommand()
  {
    return 
    Commands.sequence
    (
      runSpindexerCommand(),
      Commands.waitSeconds(spindexerPulseDelay),
      stopSpindexerCommand(),
      Commands.waitSeconds(spindexerPulseDelay)

    )

    .repeatedly();
  }

  /**@return Command for extensions */
  public Command retractCommand()
  {return extension.setTargetCommand(0);}

  public Command extendCommand()
  {return extension.setTargetCommand(ExtensionConstants.maxRotations);}

  public Command extensionJostleCommand()
  {
    return 
    Commands.sequence
    (
      extendCommand(), 
      Commands.waitSeconds(extensionJostleDelay),
      retractCommand(),
      Commands.waitSeconds(extensionJostleDelay)
    )
    .repeatedly();
  }
  @Override
  public void periodic() {}
}
