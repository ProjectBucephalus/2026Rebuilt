package frc.robot.util.autobuilder;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.util.PBDash;
import frc.robot.util.autobuilder.ParsedRepr.Instruction;

/**
 * Dynamically creates an autonomous Command from an input string of instructions
 * @author 5985
 */
public class AutoBuilder 
{
  /**
   * Compiles an auto string into a sequential command
   * 
   * @param commandInput The auto string, comprised of instructions seperated by commas.
   *                     Each instruction is a name followed by parenthesis-delimited arguments. Whitespace is ignored. 
   *                     For example, {@code driveto(1 2), waitfor(3), driveto(4 5 6)}
   * @param swerveStateSup Swerve state supplier, used for instructions involving driving or the robot's position
   * @param state The robot's state object, used for instructions such as setting auto-passing
   * @param s_Swerve The swerve subsystem
   * @param s_Intake The intake subsystem
   * @return A command that executes the auto string's instructions in sequence
   */
  public static Command compile
  (
    String source,
    Supplier<SwerveDriveState> swerveStateSup,
    RobotState state,
    CommandSwerveDrivetrain s_Swerve, 
    Intake s_Intake
  )
  {
    PBDash.AUTO_ERRS.init();
    // driveto(1 2) becomes [Token(Text, "driveto"), Token(LParen, "("), Token(Num, "1"), Token(Num, "2"), Token(LParen, ")")]
    List<Token> tokens = new Tokeniser(PBDash.AUTO_STRING.get()).tokenise(); 
    // [Token(Text, "driveto"), Token(LParen, "("), Token(Num, "1"), Token(Num, "2"), Token(LParen, ")")] 
    // becomes [Instruction(driveto, [Value(Num, 1), Value(Num, 1)])]
    List<Instruction> instrs = new Parser(tokens).parse(); 
    return new CommandGen(instrs, swerveStateSup, state, s_Swerve, s_Intake).build();
  }

  /** Appends the provided error message to {@link PBDash#AUTO_ERRS}, followed by a comma */
  protected static void error(String message) 
  {
    PBDash.AUTO_ERRS.append(message + ", ");
  }  
}
