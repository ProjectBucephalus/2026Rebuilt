package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.generic.BinaryMotor;
import frc.robot.subsystems.generic.LimitedMotor;
import static frc.robot.constants.Constants.HopperConstants.*;
import static frc.robot.constants.Constants.HopperConstants.ExtensionConstants.extensionJostleDelay;

/**
 * Ball processing master-system with extendable intake, internally creates and manages associated subsystems
 * @author 5985
 */
public class Hopper extends SubsystemBase 
{
  private BinaryMotor spindexer;
  private BinaryMotor intake;
  private LimitedMotor extension;
  
  /**
   * Creates a ball processing master-system with extendable intake, internally creates and manages associated subsystems
   * @param processorCAN CAN-ID for processor motor (e.g. belt-grid, spindexer, etc.)
   * @param intakeCAN CAN-ID for intake motor
   * @param extensionCAN CAN-ID for intake-extension motor
   * @param extensionLimitIO DIO-ID of extension home switch
   */
  public Hopper(int processorCAN, int intakeCAN, int extensionCAN, int extensionLimitIO)
  { 
    spindexer = new BinaryMotor(processorCAN, SpindexerConstants.spindexerSpeed, SpindexerConstants.spindexerConfig);
    intake = new BinaryMotor(intakeCAN, IntakeConstants.intakeSpeed, IntakeConstants.intakeConfig);
    extension = new LimitedMotor(extensionCAN, extensionLimitIO, 0, ExtensionConstants.maxRotations, ExtensionConstants.extensionConfig);
  }
  
  /** @return Command to start running intake at default speed */
  public Command runIntakeCommand()
  {return intake.startCommand();}

  /** @return Command to stop the intake */
  public Command stopIntakeCommand()
  {return intake.stopCommand();}
  
  /** @return Command to start running spindexer at default speed */
  public Command runSpindexerCommand()
  {return spindexer.startCommand();}

  /** @return Command to stop the spindexer */
  public Command stopSpindexerCommand()
  {return spindexer.stopCommand();}

  /** @return Command to continually pulse the spindexer to agitate gamepieces */
  public Command pulseSpindexerCommand()
  {
    return 
    Commands.sequence
    (
      runSpindexerCommand(),
      //Waits for 0.25 seconds
      Commands.waitSeconds(SpindexerConstants.spindexerPulseDelay),
      stopSpindexerCommand(),
      Commands.waitSeconds(SpindexerConstants.spindexerPulseDelay)
    )
    .repeatedly();
  }

  /** @return Command to retract the extension to home */
  public Command retractCommand()
  {return extension.setTargetCommand(0);}

  /** @return Command to extend the extension to max */
  public Command extendCommand()
  {return extension.setTargetCommand(ExtensionConstants.maxRotations);}

  /** @return Command to continually jostle the extension to agitate gamepieces */
  public Command extensionJostleCommand()
  {
    return 
    Commands.sequence
    (
      retractCommand(),
      //Waits for 0.25 seconds
      Commands.waitSeconds(extensionJostleDelay),
      extendCommand(), 
      Commands.waitSeconds(extensionJostleDelay)
    )
    .repeatedly();
  }

  /** @return Command to activate all systems */
  public Command deployAllCommand()
  {
    return
    Commands.parallel
    (
      runIntakeCommand(),
      runSpindexerCommand(),
      extendCommand()
    );
  }

  /** @return Command to stow all systems */
  public Command stowAllCommand()
  {
    return
    Commands.parallel
    (
      stopIntakeCommand(),
      stopSpindexerCommand(),
      retractCommand()
    );
  }

  @Override
  public void periodic() {}
}
