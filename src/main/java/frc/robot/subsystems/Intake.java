package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants.RollerConstants;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.generic.VelocityMotor;
import frc.robot.util.PBDash;

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
  private final VelocityMotor roller = new VelocityMotor(IDConstants.intakeCAN, RollerConstants.intakeConfig);

  public Intake() {}

  /** @return Command to stop the intake */
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

    roller.setSpeed(rollerSpeed);
  }

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
    {return roller.devicesValid();}
}
