package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.HopperConstants.ExtensionConstants;
import frc.robot.constants.Constants.HopperConstants.IntakeConstants;
import frc.robot.constants.Constants.HopperConstants.SpindexerConstants;
import frc.robot.subsystems.generic.BinaryMotor;
import frc.robot.subsystems.generic.LimitedMotor;
import static frc.robot.constants.Constants.HopperConstants.*;
import static frc.robot.constants.Constants.HopperConstants.ExtensionConstants.extensionJostleDelay;

import java.security.cert.Extension;
import java.util.function.DoubleSupplier;

/**
 * Ball processing master-system with extendable intake, internally creates and manages associated subsystems
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Hopper extends SubsystemBase 
{
  @Logged
  private BinaryMotor spindexer;
  @Logged
  private BinaryMotor intake;
  @Logged
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
    extension = new LimitedMotor(extensionCAN, extensionLimitIO, ExtensionConstants.minRotations, ExtensionConstants.maxRotations, ExtensionConstants.homeRotations, ExtensionConstants.extensionConfig);
  }
  
  /** @return Command to start running intake at default speed */
  public Command startIntakeCommand()
  {return intake.startCommand();}

  /** @return Command that runs the intake until it is interrupted */
  public Command runIntakeCommand()
  {return intake.runCommand();}
  
  /** @return Command to start running intake at negative default speed */
  public Command reverseIntakeCommand()
  {return intake.reverseCommand();}

  /** @return Command to stop the intake */
  public Command stopIntakeCommand()
  {return intake.stopCommand();}
  
  /** @return Command to start running spindexer at default speed */
  public Command runSpindexerCommand()
  {return spindexer.startCommand();}
  
  /** @return Command to start running spindexer at negative default speed */
  public Command reverseSpindexerCommand()
  {return spindexer.reverseCommand();}

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
  {return extension.setTargetCommand(ExtensionConstants.minRotations);}

  /** @return Command to extend the extension to max */
  public Command extendCommand()
  {return extension.setTargetCommand(ExtensionConstants.maxRotations);}

  /**
   * @param  shiftSup Supplier for relative control value, mechanism rotations
   * @return Command to smoothly control the extension 
   */
  public Command manualExtensionCommand(DoubleSupplier shiftSup)
    {return extension.adjustTargetCommand(shiftSup);}

  public boolean extended() 
    {return MathUtil.isNear(ExtensionConstants.maxRotations, extension.getAngle(), ExtensionConstants.extendedTolerance);}

  /** @return Command to continually jostle the extension to agitate gamepieces */
  public Command extensionJostleCommand()
  {
    return 
    Commands.sequence
    (
      extension.setTargetCommand(-0.2),
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
      startIntakeCommand(),
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
