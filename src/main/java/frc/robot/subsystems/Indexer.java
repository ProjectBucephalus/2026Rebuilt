// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.FeederConstants;
import frc.robot.constants.Constants.FeederConstants.SpindexerConstants;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.generic.BiMotor;
import frc.robot.subsystems.generic.VelocityMotor;

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
  private final VelocityMotor spindexer = new VelocityMotor
  (
    IDConstants.spindexerCAN, 
    SpindexerConstants.spindexerConfig
  );

  /** Creates a new Indexer. */
  public Indexer() {}

  /** @return Command to run the indexer. Automatically stops when the command ends */
  public Command runCommand(DoubleSupplier speedSup)
  {
    return parallel
    (
      spindexer.runCommand(speedSup),
      feeder.runCommand(speedSup)
    )
    .withName("Run Indexer");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
