// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.FeederConstants;
import frc.robot.constants.Constants.HopperConstants.SpindexerConstants;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.generic.BiMotor;
import frc.robot.subsystems.generic.BinaryMotor;

public class Indexer extends SubsystemBase 
{
  @Logged
  private final BiMotor feeder = new BiMotor
  (
    IDConstants.portFeederCan,
    IDConstants.stbdFeederCan,
    FeederConstants.feederConfig,
    false
  );
  @Logged
  private final BinaryMotor spindexer = new BinaryMotor
  (
    IDConstants.spindexerCAN, 
    SpindexerConstants.spindexerSpeed, 
    SpindexerConstants.spindexerConfig
  );

  /** Creates a new Indexer. */
  public Indexer() 
  {
  }

  /** @return Command to start running spindexer at default speed */
  public Command runCommand()
  {
    return new ParallelCommandGroup
    (
      spindexer.runCommand(),
      feeder.setSpeedCommand(0)
    );
  }
  
  /** @return Command to start running spindexer at negative default speed */
  public Command reverseSpindexerCommand()
  {return spindexer.reverseCommand();}

  /** @return Command to stop the spindexer */
  public Command stopSpindexerCommand()
  {return spindexer.stopCommand();}

  // /** @return Command to continually pulse the spindexer to agitate gamepieces */
  // public Command pulseSpindexerCommand()
  // {
  //   return 
  //   Commands.sequence
  //   (
  //     runSpindexerCommand(),
  //     //Waits for 0.25 seconds
  //     Commands.waitSeconds(SpindexerConstants.spindexerPulseDelay),
  //     stopSpindexerCommand(),
  //     Commands.waitSeconds(SpindexerConstants.spindexerPulseDelay)
  //   )
  //   .repeatedly();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
