package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants.RollerConstants;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.generic.VelocityMotor;

import java.util.function.Supplier;

/**
 * Ball processing master-system with extendable intake, internally creates and manages associated subsystems
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Intake extends SubsystemBase 
{
  public static enum RollerState { On, Off, Reversed }

  public RollerState state = RollerState.Off;
  private double rollerSpeed;
  private final Supplier<SwerveDriveState> swerveStateSup;

  public Intake( Supplier<SwerveDriveState> swerveStateSup)
  {
    this.swerveStateSup = swerveStateSup;
  }

  @Logged
  private final VelocityMotor roller = new VelocityMotor(IDConstants.intakeCAN, RollerConstants.intakeConfig);

  /** @return Command to stop the intake */
  public Command setStateCmd(RollerState state)
    {return roller.runOnce(() -> this.state = state);}

  @Override
  public void periodic() 
  {
    switch (state)  
    {
      case On -> 
      {
        rollerSpeed = (swerveStateSup.get().Speeds.vxMetersPerSecond * RollerConstants.intakeMinSpeed);

        if (rollerSpeed > RollerConstants.intakeMaxSpeed) 
        {
          rollerSpeed = RollerConstants.intakeMaxSpeed;
        }

        roller.setSpeed(rollerSpeed);
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
