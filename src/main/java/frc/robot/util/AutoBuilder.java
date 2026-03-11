package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot.AutoState;
import frc.robot.commands.swerve.PathFollowDrive;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;

/**
 * Dynamically creates Command list from input string of tags
 * @author 5985
 */
public class AutoBuilder 
{
  /**
   * An auto instruction. Used as an intermediate representation between the text string and the final command output
   */
  private static record Instruction(char code, int... args) 
  {
    public int argCount()
      {return args.length;}
    
    public int arg(int i)
      {return args[i];}

    public boolean boolArg(int i)
      {return args[i] != 0;}

    /**
     * Attempts to create an Instruction, returning Empty if the provided text is invalid
     * 
     * @param input The string to be parsed as an Instruction
     * @return Empty in any of the following cases: <ul>
     *          <li> The input is null
     *          <li> The input's length is 0
     *          <li> Any of the operands in the instruction input cannot be parsed as ints 
     *          </ul>
     * <li>    Non-empty containing the instruction parsed from the input otherwise
     */
    private static Optional<Instruction> parse(String input)
    {
      if (input == null || input.length() == 0) return Optional.empty();
      
      input = input.replaceAll("//s", "").toLowerCase();
      char code = input.charAt(0);
      
      if (input.length() > 1)
      {
        String[] inArgs = input.substring(1).trim().split(":");
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
   * @param errHandler A consumer to accept any erroneous instructions, intended for error logging purposes
   * @return The input parsed into a list of instructions, minus any invalid instructions
   */
  private static List<Instruction> parseInstructions(String input) 
  {
    String[] splitInput = input.split(",");
    var out = new ArrayList<Instruction>(splitInput.length);

    for (var instr : splitInput) 
    {
      Instruction
        .parse(instr.trim())
        .ifPresentOrElse
        (
          out::add, 
          () -> PBDash.AUTO_ERRS.put(PBDash.AUTO_ERRS.get() + instr + ", ")
        );
    }

    return out;
  }

  /**
   * Compiles an auto string into a sequential command
   * 
   * @param commandInput The auto string, comprised of instructions seperated by commas.
   *                     Each instruction is an opcode character followed by any number of colon-seperated operands. Whitespace is ignored. 
   *                     For example, {@code g 1:2, w3, g4:5:6}
   * @param s_Swerve The swerve subsystem, used for instructions involving driving
   * @param swerveStateSup Swerve state supplier, used for instructions involving driving or the robot's position
   * @param errHandler A consumer to accept any erroneous instructions, intended for error logging purposes
   * @return A command that executes the auto string's instructions in sequence
   */
  public static Command compileAutoString
  (
    String commandInput, 
    Supplier<SwerveDriveState> swerveStateSup,
    AutoState autoControl,
    CommandSwerveDrivetrain s_Swerve, 
    Hopper s_Intake
  )
  {
    var currPose = swerveStateSup.get().Pose;
    // The command list to be output
    var commandList = new SequentialCommandGroup();
    // For each instruction, adds the corresponding commands to the list
    for (var instr : parseInstructions(commandInput)) 
    {
      switch (instr.code) 
      {
        // g x:y:r - Go to x, y, r (alliance origin relative). r optional
        case 'g' ->
				{         
          var posTarget = new Translation2d
          (
            MathUtil.clamp(instr.arg(0), 0.5, (FieldConstants.fieldCentre.getX()) - 0.5), 
            MathUtil.clamp(instr.arg(1), 0.5, FieldConstants.fieldWidth - 0.5)
          );
          var rotationTarget = 
            instr.argCount() > 2 ? 
            Rotation2d.fromDegrees(instr.arg(2)) : 
            currPose.getRotation();

          currPose = new Pose2d(posTarget, rotationTarget);

          commandList.addCommands(new PathFollowDrive(s_Swerve, swerveStateSup, FieldUtils.allianceRotatePose(currPose)));
        }

        case 'f' ->
          commandList.addCommands(new PathFollowDrive(s_Swerve, swerveStateSup, Path.autoPaths[instr.arg(0)]));

        // w d - Wait for duration d
        case 'w' -> 
          commandList.addCommands(Commands.waitSeconds(instr.arg(0)));

        // t d - wait until time d
        case 't' ->
          commandList.addCommands(Commands.waitUntil(() -> Timer.getMatchTime() < (15 - instr.arg(0))));

        case 'i' ->
        {
          if (instr.boolArg(0)) 
            commandList.addCommands(Commands.parallel(s_Intake.extendCommand(), s_Intake.startIntakeCommand()));
          else 
            commandList.addCommands(s_Intake.stopIntakeCommand());
        }

        case 'p' -> 
          commandList.addCommands(Commands.runOnce(() -> autoControl.pass = instr.boolArg(0)));
      }
    }

    return commandList.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}
