package frc.robot.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRoller;

public class AutoFactories 
{
  /**
   * Splits a string of auto command phrases and gets the path command and robot command associated with each command phrase
   * @param commandInput The string of commands to split, seperated by commas with no spaces (e.g. "a1,rA1,p,cR3")
   * @return An array of commands, from the input command phrase string, in the same order
   */
  public static Command getCommandList(String commandInput, CoralRoller s_Coral, CommandSwerveDrivetrain s_Swerve, Supplier<SwerveDriveState> swerveStateSup)
  {
    // Removes all whitespace characters from the single-String command phrases, ensures it's all lowercase, and then splits it into individual strings, which are stored in an array
    String[] splitCommands = commandInput.replaceAll("//s", "").toLowerCase().split(",");
    // The commands produced to be run
    SequentialCommandGroup commandList = new SequentialCommandGroup();

    // For each command phrase, adds the associated path and then the associated command to the command list
    for (String splitCommand : splitCommands) 
    {
      switch (splitCommand.charAt(0)) 
      {
        case 'g' ->
				{
          int seperatorIndex = splitCommand.indexOf(":");
          
          Translation2d posTarget = new Translation2d
          (
            MathUtil.clamp(Double.parseDouble(splitCommand.substring(1, seperatorIndex)), 0.5, (FieldConstants.fieldCentre.getX()) - 0.5), 
            MathUtil.clamp(Double.parseDouble(splitCommand.substring(seperatorIndex + 1)), 0.5, FieldConstants.fieldWidth - 0.5)
          );

          Rotation2d rotationTarget = 
            splitCommand.contains(";") ? 
            new Rotation2d(Units.degreesToRadians(Double.parseDouble(splitCommand.substring(splitCommand.indexOf(";"))))) : 
            swerveStateSup.get().Pose.getRotation().plus(Rotation2d.k180deg);
          
          commandList.addCommands(s_Swerve.poseDriveCommand(new AlliancePose2dSup(posTarget, rotationTarget), swerveStateSup));
        }

        case 'w' -> 
          commandList.addCommands(Commands.waitSeconds(Double.parseDouble(splitCommand.substring(1))));

        case 't' ->
				{
          double targetMatchTimeElapsed = Double.parseDouble(splitCommand.substring(1));
          commandList.addCommands(Commands.waitUntil(() -> Timer.getMatchTime() < (15 - targetMatchTimeElapsed)));
        }

        case 'r' ->
          commandList.addCommands(
            s_Swerve.poseDriveCommand(new AlliancePose2dSup(FieldConstants.getLineup(splitCommand)), swerveStateSup),
            Commands.waitSeconds(0.1),
            s_Coral.setSpeedCommand(Constants.Coral.forwardSpeed).until(s_Coral::getSensor)
          );

        case 'c' ->
          commandList.addCommands(
            s_Swerve.poseDriveCommand(new AlliancePose2dSup(FieldConstants.getLineup(splitCommand)), swerveStateSup),
            Commands.waitUntil(s_Coral::getSensor)
          );
      }
    }

    return commandList.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}
