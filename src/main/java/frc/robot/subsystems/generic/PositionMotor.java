package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class PositionMotor extends SubsystemBase
{
  protected final TalonFX m_Position;

  protected final MotionMagicVoltage request = new MotionMagicVoltage(0);

  /**
   * Creates a wrapper around a TalonFX to provide velocity control
   * 
   * @param id the id of the motor
   * @param config the config to apply to the wrapped motor
   */
  public PositionMotor(int id, TalonFXConfiguration config) 
  {
    m_Position = new TalonFX(id);
    m_Position.getConfigurator().apply(config);
  }

  /**
   * Sets the target point for the motor 
   * @param target mechanism rotations
   */
  public void setTarget(double pos)
    {baseSetTarget(pos);}

  /**
   * Sets the target point for the motor, can't be overridden
   * Exists as a sort of hacky solution to get around overriden versions of the method
   * @param target mechanism rotations
   */
  private void baseSetTarget(double pos)
    {m_Position.setControl(request.withPosition(pos));}

  /**
   * Creates a command to set the target point for the motor <p>
   * @param targetSup mechanism rotations
   * @return the Command
   */
  public Command setTargetCmd(DoubleSupplier targetSup)
    {return runOnce(() -> setTarget(targetSup.getAsDouble()));}

  /**
   * Creates a command to set the target point for the motor <p>
   * NOTE: The provided value is only evaluated when the command is created
   * @param target mechanism rotations
   * @return the Command
   */
  public Command setTargetCmd(double target)
    {return setTargetCmd(() -> target);}

  /**
   * Creates a command to continuously adjust the target point of the motor by a dynamic amount <p>
   * Primarily intended for joystick control
   * @param shiftSup A supplier for the amount to adjust the target by in mechanism rotations
   * @return the Command
   */
  public Command adjustTargetCmd(DoubleSupplier shiftSup) 
  {
    return new Command() 
    {
      double lastInput;

      @Override
      public void execute() 
      {
        if (shiftSup.getAsDouble() != 0 || lastInput != 0) 
        {
          lastInput = shiftSup.getAsDouble();
          baseSetTarget(getAngle() + lastInput);
        }
      }
    };
  }

  /** @return Current angle of the motor, in mechanism rotations */
  @Logged(name = "angle Rotations")
  public double getAngle() 
    {return m_Position.getPosition().getValue().in(Units.Rotations);}
}
