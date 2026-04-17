package frc.robot.autobuilder;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.DriveBuilder;
import frc.robot.Robot.RobotState;
import frc.robot.autobuilder.ParsedRepr.*;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.IntakeConstants.ExtensionConstants;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Path;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.RollerState;
import frc.robot.subsystems.generic.LinearExtension;
import frc.robot.subsystems.generic.PositionMotor;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;

/** Builds the final auto command from a list of {@link ParsedRepr.Instruction Instructions} */
public class CommandGen 
{
  /* Robot related values used by the produced command */
  private final Intake s_Intake;
  private final PositionMotor s_Extension;
  private final LinearExtension s_Climber;
  private final DigitalInput io_ClimberPost;
  private final RobotState state;

  /** The final command group that gets built from the instructions */
  private SequentialCommandGroup commands;
  /** The current instruction being compiled */
  private Instruction instr;
  /** Used to track the pose the robot will be at over the course of the auto */
  private Pose2d currPose;

  /**
   * Creates a new command generator, storing all the provided robot values internally for use in the produced command
   * @param instrs The instructions to be compiled
   * @param s_Swerve The swerve subsystem
   * @param s_Intake The intake subsystem
   * @param s_Climber The climber subsystem
   * @param state The robot's state object
   */
  public CommandGen
  (
    Intake s_Intake,
    PositionMotor s_Extension,
    LinearExtension s_Climber,
    DigitalInput io_ClimberPost,
    RobotState state
  )
  {
    this.s_Intake = s_Intake;
    this.s_Extension = s_Extension;
    this.s_Climber = s_Climber;
    this.io_ClimberPost = io_ClimberPost;
    this.state = state;
  }

  /**
   * Runs the actual compilation process, compiling each instruction into a command and adding that to the final command group
   * @return The final auto command
   */
  public Command compile(List<Instruction> instrs)
  {
    commands = new SequentialCommandGroup();
    currPose = state.swerve.Pose;

    // Iterate over each instruction, calling a seperate function that handles the actual compilation logic and handling any errors that arise
    // This design means that the actual compilation logic is seperated from the error handling, and doesn't have to consider them
    for (int pos = 0; pos < instrs.size(); pos++)
    {
      instr = instrs.get(pos);
      try 
      {
        compileInstr();
      }
      catch (TypeMismatchException e)
      {
        // Example output: "argument `false` of instruction 0 (`intake`) has type Text but should have type Bool"
        AutoBuilder.error
        (
          "argument `", 
          e.found.value(), 
          "` of instruction ",
          pos + 1, 
          " (`", 
          instr.type(), 
          "`) has type ", 
          e.found.type(), 
          " but should have type ", 
          e.expected
        );
      }
      catch (ArgCountException e) 
      {
        // Example output: "instruction 0 (`follow`) has 0 arguments but should have 1 arguments"
        AutoBuilder.error
        (
          "instruction ",
          pos + 1, 
          " (`", 
          instr.type(), 
          "`) has  ",
          e.found,
          " arguments but should have ",
          e.expected,
          " arguments"
        );
      }
      catch (GeneralException e) 
      {
        AutoBuilder.error(e.msg);
      }
    }

    return commands.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Performs the actual compilation logic for each instruction type, seperate from the error handling done in {@link #compile()} <p>
   * 
   * Most of the errors are handled as exceptions, which are thrown by helper methods
   * and implicitly get rethrown by this to be caught in {@link #compile()}
   */
  private void compileInstr() throws TypeMismatchException, ArgCountException, GeneralException
  {
    switch (instr.type())
    {
      // driveto x y r - Go to pose `x`, `y`, `r` (alliance origin relative). `r` optional, maintains current rotation if omitted
      case driveto -> 
      { 
        Rotation2d rotationTarget;
        // Some extra handling is required due to the optional argument
        if (instr.args().length > 2) 
        {
          assertArgCount(3);
          rotationTarget = Rotation2d.fromDegrees(instr.arg(2).asNum());
        }
        else
        {
          assertArgCount(2);
          rotationTarget = currPose.getRotation();
        }

        // Clamp the target pose to at least half a meter from the field walls and the midline for safety and to handle mis-inputs
        // If either value changes as a result of this clamping, we provide a warning but still continue
        double xArg = instr.arg(0).asNum();
        double x = MathUtil.clamp(xArg, 0.5, (FieldConstants.fieldCentre.getX()) - 0.5);
        if (x != xArg) {AutoBuilder.error("warning: x value `", xArg, "` was clamped to `", x, "`");}
        double yArg = instr.arg(1).asNum();
        double y = MathUtil.clamp(yArg, 0.5, FieldConstants.fieldWidth - 0.5);
        if (y != yArg) {AutoBuilder.error("warning: y value `", yArg, "` was clamped to `", y, "`");}

        currPose = new Pose2d(new Translation2d(x, y), rotationTarget);
        Pose2d targetPose = FieldUtils.allianceRotatePose(currPose);

        PBDash.addToFieldObject("Auto Path", targetPose);

        // All prior handling was done using a blue alliance origin pose, and we now rotate the pose to match our actual alliance
        commands.addCommands(DriveBuilder.pathFollow(FieldUtils.allianceRotatePose(currPose)));
      }
      // driveby x y - Relative drive
      case driveby -> 
      {
        assertArgCount(2);

        Translation2d offset = new Translation2d(instr.arg(0).asNum(), instr.arg(1).asNum());
        currPose = new Pose2d(currPose.getTranslation().plus(offset), currPose.getRotation());
        Pose2d targetPose = FieldUtils.allianceRotatePose(currPose);

        PBDash.addToFieldObject("Auto Path", targetPose);

        // All prior handling was done using a blue alliance origin pose, and we now rotate the pose to match our actual alliance
        commands.addCommands(DriveBuilder.pathFollow(targetPose));
      }
      // follow n - Follow the path with name `n` in Path.autoPaths
      case follow -> 
      {
        assertArgCount(1);

        var pathName = instr.arg(0).asText();
        var path = Path.autoPaths.get(pathName);

        if (path == null) throw new GeneralException("no path `" + pathName + "`");

        path.display("Auto Path");

        currPose = path.targetPose();
        commands.addCommands(DriveBuilder.pathFollow(path.allianceRotated()));
      }
      // waitfor d - Wait for duration `d`
      case waitfor -> 
      {
        assertArgCount(1);

        commands.addCommands(Commands.waitSeconds(instr.arg(0).asNum()));
      }
      // waituntil d - wait until time `d`
      case waituntil -> 
      {
        assertArgCount(1);

        double duration = instr.arg(0).asNum();
        commands.addCommands(Commands.waitUntil(() -> Timer.getMatchTime() < (15 - duration)));
      }
      // intake b - if `b` is `on`/true, deploys and runs intake. if `b` is `off`/false, stops intake (leaving it deployed)
      case intake -> 
      {
        assertArgCount(1);

        if (instr.arg(0).asBool()) 
          commands.addCommands(Commands.parallel(s_Extension.setTargetCmd(() -> ExtensionConstants.maxRotations), s_Intake.setStateCmd(RollerState.On)));
        else 
          commands.addCommands(s_Intake.setStateCmd(RollerState.Off));
      }
      // passing b - sets auto passing on or off based on `b`
      case passing -> 
      {
        assertArgCount(1);

        boolean passState = instr.arg(0).asBool();
        commands.addCommands(Commands.runOnce(() -> PBDash.IO_SHOOT_PASS.put(passState)));
      }
      case climb ->
      {
        assertArgCount(1);

        String text = instr.arg(0).asText();
        boolean isLeft = switch (text)
        {
          case "left" -> true;
          case "right" -> false;
          default -> throw new GeneralException("expected the argument to be `left` or `right`, but it was ", text);
        };

        Pose2d climbStartPose = isLeft ? GeoFencing.climbStartPoseLeft.get() : GeoFencing.climbStartPoseRight.get();
        Pose2d climbEndPose = isLeft ? GeoFencing.climbEndPoseLeft.get() : GeoFencing.climbEndPoseRight.get();

        commands.addCommands
        (
          Commands.parallel
          (
            DriveBuilder.pathFollow(climbStartPose),
            s_Climber.extendCmd()
          ),
          Commands.waitUntil(() -> !io_ClimberPost.get()),
          DriveBuilder.pathFollow(climbEndPose, () -> 0.75),
          Commands.waitUntil(io_ClimberPost::get),
          s_Climber.setTargetCmd(ClimberConstants.climbPosition)
        );
      }
    }
  }

  /**
   * Throws an exception if the number of arguments in the current instruction is different from the provided value.
   * Used as a helper to allow ergonomic, one-line checking of argument counts
   * @param count How many arguments the current instruction should have
   * @throws ArgCountException
   */
  private void assertArgCount(int count) throws ArgCountException
  {
    if (instr.args().length != count) 
      throw new ArgCountException(count, instr.args().length);
  }
}
