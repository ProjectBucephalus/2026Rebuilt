package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.PathFollowDrive;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Dynamically creates Command list from input string of tags
 * @author 5985
 */
public class AutoBuilder 
{
  private static record Instruction(char code, int... args) 
  {
    private static Optional<Instruction> parse(String input)
    {
      if (input == null || input.length() == 0) return Optional.empty();
      
      input = input.replaceAll("//s", "").toLowerCase();
      char code = input.charAt(0);
      
      if (input.length() > 1)
      {
        String[] inArgs = input.substring(1).split(":");
        int[] outArgs = new int[inArgs.length];

        for (int i = 0; i < inArgs.length; i++)
        {
          try 
            {outArgs[i] = Integer.parseInt(inArgs[i]);}
          catch (NumberFormatException e) 
            {return Optional.empty();}
        }

        return Optional.of(new Instruction(code, outArgs));
      } 
      else 
        return Optional.of(new Instruction(code));
    }
  }

  /**
   * Parses an auto string into a list of instructions, for easier inspection during command generation
   * Any invalid instructions are skipped
   * 
   * @param input The input auto string
   * @return
   */
  private static List<Instruction> parseInstructions(String input, Consumer<String> errHandler) 
  {
    String[] splitInput = input.split(",");
    var out = new ArrayList<Instruction>(splitInput.length);

    for (var instr : splitInput) 
    {
      Instruction
        .parse(instr)
        .ifPresentOrElse
        (
          out::add, 
          () -> errHandler.accept(instr)
        );
    }

    return out;
  }

  /**
   * Splits a string of auto command phrases and gets the path command and robot command associated with each command phrase
   * @param commandInput The string of commands to split, seperated by commas with no spaces (e.g. "a1,rA1,p,cR3")
   * @return An array of commands, from the input command phrase string, in the same order
   */
  public static Command compileAutoString
  (
    String commandInput, 
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<SwerveDriveState> swerveStateSup, 
    Consumer<String> errHandler
  )
  {
    // The command list to be output
    var commandList = new SequentialCommandGroup();

    // For each instruction, adds the corresponding commands to the list
    for (var instr : parseInstructions(commandInput, errHandler)) 
    {
      switch (instr.code) 
      {
        // g x:y:r - Go to x, y, r (alliance origin relative). r optional
        case 'g' ->
				{         
          var posTarget = new Translation2d
          (
            MathUtil.clamp(instr.args[0], 0.5, (FieldConstants.fieldCentre.getX()) - 0.5), 
            MathUtil.clamp(instr.args[1], 0.5, FieldConstants.fieldWidth - 0.5)
          );

          var rotationTarget = 
            instr.args.length > 2 ? 
            Rotation2d.fromDegrees(instr.args[2]) : 
            swerveStateSup.get().Pose.getRotation().plus(Rotation2d.k180deg);
          
          commandList.addCommands(new PathFollowDrive(s_Swerve, swerveStateSup, new AlliancePose2d(posTarget, rotationTarget).get()));
        }

        // w d - Wait for duration d
        case 'w' -> 
          commandList.addCommands(Commands.waitSeconds(instr.args[0]));

        // t d - wait until time d
        case 't' ->
          commandList.addCommands(Commands.waitUntil(() -> Timer.getMatchTime() < (15 - instr.args[0])));
      }
    }

    return commandList.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}
