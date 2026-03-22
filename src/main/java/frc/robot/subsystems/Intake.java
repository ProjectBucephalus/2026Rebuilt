package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.generic.PositionMotor;
import frc.robot.subsystems.generic.VelocityMotor;
import frc.robot.util.Conversions;

import static frc.robot.constants.Constants.IntakeConstants.*;

/**
 * Ball processing master-system with extendable intake, internally creates and manages associated subsystems
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Intake extends SubsystemBase 
{
  public static enum RollerState { On, Off, Reversed }

  public RollerState state = RollerState.Off;

  @Logged
  private final VelocityMotor roller;
  @Logged
  private final PositionMotor extension;
  
  /**
   * Creates a ball processing master-system with extendable intake, internally creates and manages associated subsystems
   * @param processorCAN CAN-ID for processor motor (e.g. belt-grid, spindexer, etc.)
   * @param intakeCAN CAN-ID for intake motor
   * @param extensionCAN CAN-ID for intake-extension motor
   */
  public Intake(int intakeCAN, int extensionCAN)
  { 
    roller = new VelocityMotor(intakeCAN, RollerConstants.intakeConfig);
    extension = new PositionMotor(extensionCAN, ExtensionConstants.extensionConfig);
  }

  /** @return brake value to apply when intake is running */
  public double brakeFromIntake()
  {
    return Conversions.clamp((roller.getSpeed() - RollerConstants.brakeSpeedStart) / RollerConstants.brakeSpeedRange, 0, 1) * RollerConstants.intakeBrake;
  }
  
  /** @return Command that runs the intake until it is interrupted */
  public Command runIntakeCommand()
    {return roller.startEnd(() -> state = RollerState.On, () -> state = RollerState.Off).withName("Run Intake");}
  
  /** @return Command to start running intake at default speed */
  public Command startIntakeCommand()
    {return roller.runOnce(() -> state = RollerState.On);}

  /** @return Command to stop the intake */
  public Command stopIntakeCommand()
    {return roller.runOnce(() -> state = RollerState.Off);}

  /** @return Command to start running intake at negative default speed */
  public Command reverseIntakeCommand()
    {return roller.runOnce(() -> state = RollerState.Reversed);}

  public Command bumpSafeCommand()
  {
    return extension.setTargetCommand(() -> Math.min(ExtensionConstants.bumpSafeRotations, extension.getAngle()));
  }

  /** @return Command to retract the extension to home */
  public Command stowCommand()
    {return extension.setTargetCommand(() -> ExtensionConstants.minRotations);}

  /** @return Command to extend the extension to max */
  public Command deployCommand()
    {return extension.setTargetCommand(() -> ExtensionConstants.maxRotations);}

  /**
   * @param  shiftSup Supplier for relative control value, mechanism rotations
   * @return Command to smoothly control the extension 
   */
  public Command manualExtensionCommand(DoubleSupplier shiftSup)
    {return extension.adjustTargetCommand(shiftSup);}

  public boolean extended() 
    {return MathUtil.isNear(ExtensionConstants.maxRotations, extension.getAngle(), ExtensionConstants.extendedTolerance);}

  @Override
  public void periodic() 
  {
    switch (state)  
    {
      case On -> 
      {
        // TODO
      }
      case Off -> 
      {
        roller.setSpeed(0);
      }
      case Reversed -> 
      {
        roller.setSpeed(-RollerConstants.intakeSpeed);
      }
    }
  }
}
