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
import frc.robot.subsystems.generic.VelocityMotor;

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
  private final VelocityMotor intake;
  @Logged
  private final LimitedMotor extension;
  
  /**
   * Creates a ball processing master-system with extendable intake, internally creates and manages associated subsystems
   * @param processorCAN CAN-ID for processor motor (e.g. belt-grid, spindexer, etc.)
   * @param intakeCAN CAN-ID for intake motor
   * @param extensionCAN CAN-ID for intake-extension motor
   * @param extensionLimitIO DIO-ID of extension home switch
   */
  public Hopper(int intakeCAN, int extensionCAN, int extensionLimitIO)
  { 
    intake = new VelocityMotor(intakeCAN, IntakeConstants.intakeConfig);
    extension = new LimitedMotor(extensionCAN, extensionLimitIO, ExtensionConstants.minRotations, ExtensionConstants.maxRotations, ExtensionConstants.homeRotations, ExtensionConstants.extensionConfig);
  }
  
  /** @return Command to start running intake at default speed */
  public Command startIntakeCommand()
  {return intake.setSpeedCommand(() -> IntakeConstants.intakeSpeed);}

  /** @return Command that runs the intake until it is interrupted */
  public Command runIntakeCommand()
  {return intake.runCommand(() -> IntakeConstants.intakeSpeed);}
  
  /** @return Command to start running intake at negative default speed */
  public Command reverseIntakeCommand()
  {return intake.setSpeedCommand(() -> -IntakeConstants.intakeSpeed);}

  /** @return Command to stop the intake */
  public Command stopIntakeCommand()
  {return intake.setSpeedCommand(() -> 0);}

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

  @Override
  public void periodic() {}
}
