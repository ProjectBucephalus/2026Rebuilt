package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.generic.LimitedMotor;
import frc.robot.subsystems.generic.VelocityMotor;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;

import static frc.robot.constants.Constants.IntakeConstants.*;

/**
 * Ball processing master-system with extendable intake, internally creates and manages associated subsystems
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Intake extends SubsystemBase 
{
  @Logged
  private final VelocityMotor roller;
  @Logged
  private final LimitedMotor extension;
  
  /**
   * Creates a ball processing master-system with extendable intake, internally creates and manages associated subsystems
   * @param processorCAN CAN-ID for processor motor (e.g. belt-grid, spindexer, etc.)
   * @param intakeCAN CAN-ID for intake motor
   * @param extensionCAN CAN-ID for intake-extension motor
   * @param extensionLimitIO DIO-ID of extension home switch
   */
  public Intake(int intakeCAN, int extensionCAN, int extensionLimitIO)
  { 
    roller = new VelocityMotor(intakeCAN, RollerConstants.intakeConfig);
    extension = new LimitedMotor(extensionCAN, extensionLimitIO, ExtensionConstants.minRotations, ExtensionConstants.maxRotations, ExtensionConstants.homeRotations, ExtensionConstants.extensionConfig);
  }

  /** @return brake value to apply when intake is running */
  public double brakeFromIntake()
  {
    return Conversions.clamp((roller.getSpeed() - RollerConstants.brakeSpeedStart) / RollerConstants.brakeSpeedRange, 0, 1) * RollerConstants.intakeBrake;
  }
  
  /** @return Command to start running intake at input speed */
  public Command runIntakeCommand(DoubleSupplier speedSup)
    {return roller.runCommand(speedSup).withName("Manual Run Intake");}

  /** @return Command that runs the intake until it is interrupted */
  public Command runIntakeCommand()
    {return roller.runCommand(() -> RollerConstants.intakeSpeed).withName("Run Intake");}
  
  /** @return Command to start running intake at default speed */
  public Command startIntakeCommand()
    {return roller.setSpeedCommand(() -> RollerConstants.intakeSpeed);}

  /** @return Command to start running intake at negative default speed */
  public Command reverseIntakeCommand()
    {return roller.setSpeedCommand(() -> -RollerConstants.intakeSpeed);}

  /** @return Command to stop the intake */
  public Command stopIntakeCommand()
    {return roller.setSpeedCommand(() -> 0);}

  public Command bumpSafeCommand()
    {return extension.setTargetCommand(() -> Math.min(ExtensionConstants.bumpSafeRotations, extension.getAngle()));}

  /** @return Command to retract the extension to home */
  public Command retractCommand()
    {return extension.retractCommand().unless(PBDash.E_STOP::get);}

  /** @return Command to extend the extension to max */
  public Command extendCommand()
    {return extension.deployCommand().unless(PBDash.E_STOP::get);}

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
    Commands.repeatingSequence
    (
      extension.setTargetCommand(-0.2),
      Commands.waitSeconds(ExtensionConstants.extensionJostleDelay),
      extendCommand(), 
      Commands.waitSeconds(ExtensionConstants.extensionJostleDelay)
    )
    .alongWith(runIntakeCommand())
    .unless(PBDash.E_STOP::get);
  }

  @Override
  public void periodic() 
  {
    PBDash.putDouble("Intake Speed", roller.getSpeed());
  }
}
