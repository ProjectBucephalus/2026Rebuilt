package frc.robot.commands.swerve;

import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** 
 * Foundation of all other drive commands 
 * @author 5985
 */
public abstract class SwerveCommandBase extends Command
{
  protected final double deadband = Constants.ControlConstants.stickDeadband;

  protected CommandSwerveDrivetrain s_Swerve;

  protected Supplier<Translation2d> joystickSupplier;
  
  protected Translation2d motionXY;
  protected Pose2d robotPose;

  /** Creates a new SwerveCommandBase. This has no rotation or drive-request methods or objects */
  public SwerveCommandBase(CommandSwerveDrivetrain s_Swerve, Supplier<Translation2d> joystickSupplier) 
  {
    this.s_Swerve = s_Swerve;
    
    this.joystickSupplier = joystickSupplier;

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() 
  {
    initDriveConstraints();
  }

  /** Override to add additional initialization functionality */
  protected void initDriveConstraints() {}
}
