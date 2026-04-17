package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.IntakeConstants.RollerConstants;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.generic.VelocityMotor;
import frc.robot.util.PBDash;

/**
 * Ball processing master-system with extendable intake
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Intake extends VelocityMotor 
{
  public enum RollerState { On, Off, Reversed }

  @Logged
  public RollerState state = RollerState.Off;

  public Intake() 
    {super(IDConstants.intakeCAN, RollerConstants.intakeConfig);}

  /** @return Command to set the intake's state to the given value */
  public Command setStateCmd(RollerState state) 
    {return runOnce(() -> this.state = state);}

  @Override
  public void periodic() 
  {
    double rollerSpeed = switch (state)  
    {
      case On -> DriverStation.isTest() ? PBDash.TEST_INTAKE_SPEED.get() : RollerConstants.intakeMaxSpeed;
      case Off -> 0;
      case Reversed -> -RollerConstants.intakeMinSpeed;
    };

    setSpeed(rollerSpeed);
  }
}
