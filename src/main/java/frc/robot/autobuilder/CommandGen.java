package frc.robot.autobuilder;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.DriveBuilder;
import frc.robot.Robot.ClimbPosition;
import frc.robot.Robot.RobotState;
import frc.robot.autobuilder.ParsedRepr.*;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.IntakeConstants.ExtensionConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.FieldTuning;
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
  /** The next instruction is to drive */
  private boolean nextIsDrive;
  /** Used to track the pose the robot will be at over the course of the auto. ALWAYS HANDLED AS BLUE ALLIANCE */
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
    currPose = FieldUtils.allianceRotatePose(state.swerve.Pose);
    PBDash.putFieldObject("Auto Path", currPose);

    // Iterate over each instruction, calling a seperate function that handles the actual compilation logic and handling any errors that arise
    // This design means that the actual compilation logic is seperated from the error handling, and doesn't have to consider them
    for (int pos = 0; pos < instrs.size(); pos++)
    {
      var instr = instrs.get(pos);

      if (pos == instrs.size() - 1)
        nextIsDrive = false;
      else
        switch (instrs.get(pos + 1).type())
        {
          case driveto, driveby, follow, climb -> nextIsDrive = true;
          default -> nextIsDrive = false;
        }

      try 
      {
        compileInstr(instr);
      }
      catch (TypeMismatchException e)
      {
        // Example output: "argument `false` of instruction 0 (`intake`) has type Text, expected Bool"
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
          ", expected ", 
          e.expected
        );
      }
      catch (ArgCountException e) 
      {
        // Example output: "instruction 0 (`follow`) has 0 arguments, expected 1"
        AutoBuilder.error
        (
          "instruction ",
          pos + 1, 
          " (`", 
          instr.type(), 
          "`) has ",
          e.found,
          " arguments, expected ",
          e.expected
        );
      }
      catch (GeneralException e) 
      {
        AutoBuilder.error((Object[])e.msg);
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
  private void compileInstr(Instruction instr) throws TypeMismatchException, ArgCountException, GeneralException
  {
    switch (instr.type())
    {
      // driveto x y r - Go to pose `x`, `y`, `r` (alliance origin relative). `r` optional, maintains current rotation if omitted
      case driveto -> compileDriveTo(instr);
      // driveby x y - Relative drive
      case driveby -> compileDriveBy(instr);
      // follow n - Follow the path with name `n` in Path.autoPaths
      case follow -> compileFollow(instr);
      // waitfor d - Wait for duration `d`
      case waitfor, wait -> 
      {
        instr.assertArgCount(1);

        commands.addCommands(DriveBuilder.waitCommand().withTimeout(instr.arg(0).asNum()));
      }
      // waituntil d - wait until time `d`
      case waituntil -> 
      {
        instr.assertArgCount(1);

        double duration = instr.arg(0).asNum();
        commands.addCommands(DriveBuilder.waitCommand().until(() -> Timer.getMatchTime() < (15 - duration)));
      }
      // intake t - `on` deploys and runs intake
      //            `stow` stops and stows intake
      //            `agitate` sets intake to jostle position and idles roller 
      //            `off`, `reverse`, or `idle` set roller state as appropriate
      case intake -> 
      {
        instr.assertArgCount(1);
        switch (instr.arg(0).asText())
        {
          case "on"      -> commands.addCommands(Commands.parallel(s_Extension.gotoTargetCmd(() -> ExtensionConstants.maxRotations), s_Intake.setStateCmd(RollerState.On)));
          case "stow"    -> commands.addCommands(Commands.parallel(s_Extension.gotoTargetCmd(() -> ExtensionConstants.minRotations), s_Intake.setStateCmd(RollerState.Off)));
          case "idle"    -> commands.addCommands(Commands.parallel(s_Extension.setTargetCmd(() -> ExtensionConstants.maxRotations), s_Intake.setStateCmd(RollerState.Idle)));
          case "reverse" -> commands.addCommands(Commands.parallel(s_Extension.setTargetCmd(() -> ExtensionConstants.maxRotations), s_Intake.setStateCmd(RollerState.Reversed)));
          case "agitate" -> commands.addCommands(Commands.parallel(s_Extension.setTargetCmd(() -> ExtensionConstants.jostleRotations), s_Intake.setStateCmd(RollerState.Idle)));
          case "off"     -> commands.addCommands(s_Intake.setStateCmd(RollerState.Off));
          default -> 
          {
            commands.addCommands(s_Intake.setStateCmd(RollerState.Idle));
            AutoBuilder.error("warning: invalid intake state ", instr.arg(0).asText(), " was treated as 'idle'");
          }
        }
      }
      // passing b - sets auto passing on or off based on `b`
      case passing -> 
      {
        instr.assertArgCount(1);

        boolean passState = instr.arg(0).asBool();
        commands.addCommands(Commands.runOnce(() -> PBDash.IO_SHOOT_PASS.put(passState)));
      }
      case climb -> compileClimb(instr);
    }
  }

  private void compileDriveTo(Instruction instr) throws TypeMismatchException, ArgCountException 
  {
    Rotation2d rotationTarget;
    // Some extra handling is required due to the optional argument
    if (instr.args().length > 2) 
    {
      instr.assertArgCount(3);
      rotationTarget = Rotation2d.fromDegrees(instr.arg(2).asNum());
    }
    else
    {
      instr.assertArgCount(2);
      rotationTarget = currPose.getRotation();
    }

    // Clamp the target pose to at least half a meter from the field walls and the midline for safety and to handle mis-inputs
    // If either value changes as a result of this clamping, we provide a warning but still continue
    double xArg = instr.arg(0).asNum();
    double x = MathUtil.clamp(xArg, 0.5, (FieldConstants.fieldCentre.getX()) - 0.5);
    if (x != xArg) {AutoBuilder.error("warning: x = `", xArg, "` was clamped to `", x, "`");}
    double yArg = instr.arg(1).asNum();
    double y = MathUtil.clamp(yArg, 0.5, FieldConstants.fieldWidth - 0.5);
    if (y != yArg) {AutoBuilder.error("warning: y = `", yArg, "` was clamped to `", y, "`");}

    currPose = new Pose2d(new Translation2d(x, y), rotationTarget);
    Pose2d targetPose = FieldUtils.allianceRotatePose(currPose);

    PBDash.addToFieldObject("Auto Path", targetPose);
    // All prior handling was done using a blue alliance origin pose, and we now rotate the pose to match our actual alliance
    commands.addCommands(DriveBuilder.pathFollow(targetPose, nextIsDrive));
  }

  private void compileDriveBy(Instruction instr) throws TypeMismatchException, ArgCountException 
  {
    instr.assertArgCount(2);

    Translation2d offset = new Translation2d(instr.arg(0).asNum(), instr.arg(1).asNum());
    currPose = new Pose2d(currPose.getTranslation().plus(offset), currPose.getRotation());
    // All prior handling was done using a blue alliance origin pose, and we now rotate the pose to match our actual alliance
    Pose2d targetPose = FieldUtils.allianceRotatePose(currPose);

    PBDash.addToFieldObject("Auto Path", targetPose);

    commands.addCommands(DriveBuilder.pathFollow(targetPose, nextIsDrive));
  }

  private void compileFollow(Instruction instr) throws TypeMismatchException, ArgCountException, GeneralException
  {
    instr.assertArgCount(1);

    var pathName = instr.arg(0).asText();
    var path = Path.autoPaths.get(pathName);

    if (path == null) throw new GeneralException("no path `" + pathName + "`");

    currPose = path.targetPose();

    var alliancePath = path.allianceRotated();
    alliancePath.display("Auto Path");
    commands.addCommands(DriveBuilder.pathFollow(alliancePath, nextIsDrive));
  }

  private void compileClimb(Instruction instr) throws TypeMismatchException, ArgCountException, GeneralException
  {
    instr.assertArgCount(1);

    String text = instr.arg(0).asText();
    boolean isLeft = switch (text)
    {
      case "left" -> true;
      case "right" -> false;
      default -> throw new GeneralException("expected `left` or `right`, found ", text);
    };

    Path approachPath, climbPath;
    double maxHeight;

    if (isLeft)
      if(FieldUtils.isAlliance(Alliance.Blue))
      {
        approachPath = Path.climbApproachBlueLeft.allianceOffset(PBDash.TUNE_CLIMB_BL.get(), 0);
        climbPath = Path.climbBlueLeft.allianceOffset(PBDash.TUNE_CLIMB_BL.get(), 0);
        maxHeight = FieldTuning.postHeightOffsetBlueLeft;
      }
      else // if Alliance.Red
      {
        approachPath = Path.climbApproachRedLeft.allianceOffset(PBDash.TUNE_CLIMB_RL.get(), 0);
        climbPath = Path.climbRedLeft.allianceOffset(PBDash.TUNE_CLIMB_RL.get(), 0);
        maxHeight = FieldTuning.postHeightOffsetRedLeft;
      }
    else // if isRight
      if(FieldUtils.isAlliance(Alliance.Blue))
      {
        approachPath = Path.climbApproachBlueRight.allianceOffset(PBDash.TUNE_CLIMB_BR.get(), 0);
        climbPath = Path.climbBlueRight.allianceOffset(PBDash.TUNE_CLIMB_BR.get(), 0);
        maxHeight = FieldTuning.postHeightOffsetBlueRight;
      }
      else // if Alliance.Red
      {
        approachPath = Path.climbApproachRedRight.allianceOffset(PBDash.TUNE_CLIMB_RR.get(), 0);
        climbPath = Path.climbRedRight.allianceOffset(PBDash.TUNE_CLIMB_RR.get(), 0);
        maxHeight = FieldTuning.postHeightOffsetRedRight;
      }

    approachPath.display("Auto Path");
    climbPath.display("Auto Path");

    // If we're red, re-rotate it so that it's blue origin
    currPose = FieldUtils.allianceRotatePose(climbPath.targetPose());

    commands.addCommands
    (
      // Set climb position for fencing and vision
      Commands.runOnce(() -> state.climbPos = isLeft ? ClimbPosition.Left : ClimbPosition.Right),

      // Follow to approach point, wait until climber is fully extended
      DriveBuilder.pathFollow(approachPath)
        .alongWith(s_Climber.extendCmd(), Commands.runOnce(() -> PBDash.CLIMBER_STATE.put("Extended"))),
      Commands.waitUntil(io_ClimberPost::get),

      // Wiggle climber while following path into climb position
      Commands.either
      (
        Commands.repeatingSequence
        ( 
          s_Climber.gotoTargetCmd(maxHeight - ClimberConstants.wiggleOffset),
          Commands.waitSeconds(ClimberConstants.wiggleWait),
          s_Climber.gotoTargetCmd(maxHeight),
          Commands.waitSeconds(ClimberConstants.wiggleWait)
        )
        .raceWith(DriveBuilder.pathFollow(climbPath)),

        DriveBuilder.pathFollow(climbPath),

        PBDash.IO_CLIMB_WIGGLE::get
      ),
      s_Climber.extendCmd(),

      // Only attempt climb if the post is detected
      DriveBuilder.waitCommand().until(() -> !io_ClimberPost.get() || RobotBase.isSimulation()),
      Commands.runOnce(() -> PBDash.CLIMBER_STATE.put("Climb")),
      s_Climber.gotoTargetCmd(ClimberConstants.climbPosition),

      // Unset climb position ready for teleop
      Commands.runOnce(() -> state.climbPos = ClimbPosition.None)
    );
     
  }
}
