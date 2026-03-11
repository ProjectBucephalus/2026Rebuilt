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

public class AutoBuilder 
{
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
    List<Token> tokens = new Tokeniser(PBDash.AUTO_STRING.get()).tokenise();
    List<Instruction> instrs = new Parser(tokens).parse();
    return new CommandGen(instrs, swerveStateSup, state, s_Swerve, s_Intake).build();
  }

  public static void test()
  {
    PBDash.AUTO_ERRS.init();
    var tokens = new Tokeniser(PBDash.AUTO_STRING.get()).tokenise();
    var instrs = new Parser(tokens).parse();
    PBDash.putString("test", instrs.toString());
  }

  protected static void error(String message) 
  {
    PBDash.AUTO_ERRS.append(message + ", ");
  }  
}
