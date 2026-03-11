package frc.robot.util.autobuilder;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot.RobotState;
import frc.robot.commands.swerve.PathFollowDrive;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.util.FieldUtils;
import frc.robot.util.autobuilder.ParsedRepr.Instruction;
import frc.robot.util.autobuilder.ParsedRepr.TypeMismatchException;

public class CommandGen 
{
  private final Supplier<SwerveDriveState> swerveStateSup;
  private final RobotState state;
  private final CommandSwerveDrivetrain s_Swerve;
  private final Intake s_Intake;

  private final List<Instruction> source;
  private final SequentialCommandGroup commands = new SequentialCommandGroup();

  private int pos;
  private Instruction instr;

  private Pose2d currPose;

  public CommandGen
  (
    List<Instruction> instrs,
    Supplier<SwerveDriveState> swerveStateSup,
    RobotState state,
    CommandSwerveDrivetrain s_Swerve, 
    Intake s_Intake
  )
  {
    source = instrs;
    this.swerveStateSup = swerveStateSup;
    this.state = state;
    this.s_Swerve = s_Swerve;
    this.s_Intake = s_Intake;
  }

  public Command build()
  {
    currPose = swerveStateSup.get().Pose;

    for (pos = 0; pos < source.size(); pos++)
    {
      instr = source.get(pos);
      try 
      {
        switch (instr.type())
        {
          // driveto(x y r) - Go to `x`, `y`, `r` (alliance origin relative). r optional, maintains current rotation if omitted
          case driveto -> 
          { 
            Rotation2d rotationTarget;
            if (instr.args().length > 2) 
            {
              if (!checkArgCount(3)) break;
              rotationTarget = Rotation2d.fromDegrees(instr.arg(2).asNum());
            }
            else
            {
              if (!checkArgCount(2)) break;
              rotationTarget = currPose.getRotation();
            }

            var posTarget = new Translation2d
            (
              MathUtil.clamp(instr.arg(0).asNum(), 0.5, (FieldConstants.fieldCentre.getX()) - 0.5), 
              MathUtil.clamp(instr.arg(1).asNum(), 0.5, FieldConstants.fieldWidth - 0.5)
            );

            currPose = new Pose2d(posTarget, rotationTarget);

            commands.addCommands(new PathFollowDrive(s_Swerve, swerveStateSup, FieldUtils.allianceRotatePose(currPose)));
          }
          // follow(i) - Follow path at index `i` - 1 in Path.autoPaths (1-indexing)
          case follow -> 
          {
            if (!checkArgCount(1)) break;

            var pathName = instr.arg(0).asString();
            var path = Path.autoPaths.get(pathName);

            if (path == null)
            {
              AutoBuilder.error("no path `" + pathName + "`");
              break;
            }

            currPose = path.targetPose();
            commands.addCommands(new PathFollowDrive(s_Swerve, swerveStateSup, path.allianceRotated()));
          }
          // waitfor(d) - Wait for duration `d`
          case waitfor -> 
          {
            if (!checkArgCount(1)) break;

            commands.addCommands(Commands.waitSeconds(instr.arg(0).asNum()));
          }
          // waituntil(d) - wait until time `d`
          case waituntil -> 
          {
            if (!checkArgCount(1)) break;

            commands.addCommands(Commands.waitUntil(() -> Timer.getMatchTime() < (15 - instr.arg(0).asNum())));
          }
          // intake(b) - if `b` is true, deploys and runs intake. if `b` is false, stops intake (leaving it deployed)
          case intake -> 
          {
            if (!checkArgCount(1)) break;

            if (instr.arg(0).asBool()) 
              commands.addCommands(Commands.parallel(s_Intake.extendCommand(), s_Intake.startIntakeCommand()));
            else 
              commands.addCommands(s_Intake.stopIntakeCommand());
          }
          // passing(b) - sets auto passing on or off based on `b`
          case passing -> 
          {
            if (!checkArgCount(1)) break;

            commands.addCommands(Commands.runOnce(() -> state.pass = instr.arg(0).asBool()));
          }
        }
      }
      catch (TypeMismatchException e)
      {
        var builder = new StringBuilder();

        builder.append("argument `");
        builder.append(e.found.value());
        builder.append("` of instruction ");
        builder.append(pos + 1);
        builder.append(" (");
        builder.append(instr.type());
        builder.append(") has type ");
        builder.append(e.found.type());
        builder.append(" but should have type ");
        builder.append(e.expected);

        AutoBuilder.error(builder.toString());
      }
    }

    return commands;
  }

  private boolean checkArgCount(int count) 
  {
    var instr = source.get(pos);
    if (instr.args().length != count) 
    {
      var builder = new StringBuilder();

      builder.append("instruction ");
      builder.append(pos + 1);
      builder.append(" (");
      builder.append(instr.type());
      builder.append(") should have ");
      builder.append(count);
      builder.append(" arguments but has ");
      builder.append(instr.args().length);

      AutoBuilder.error(builder.toString());

      return false;
    }

    return true;
  }
}
