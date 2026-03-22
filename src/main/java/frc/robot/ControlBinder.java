package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.FieldConstants.GeoFencing.*;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot.ClimbPosition;
import frc.robot.Robot.RobotState;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.controlTransmutation.Brake;
import frc.robot.controlTransmutation.JoystickTransmuter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.RollerState;
import frc.robot.subsystems.generic.LinearExtension;

public record ControlBinder
(
  RobotState state,
  Supplier<SwerveDriveState> swerveStateSup,
  CommandXboxController driver,
  CommandXboxController operator,
  CommandGenericHID switchboard,
  JoystickTransmuter driverStick,
  Brake driverBrake,
  CommandSwerveDrivetrain s_Swerve,
  Vision s_Vision,
  Shooter s_PortShooter,
  Shooter s_StbdShooter,
  Intake s_Intake,
  LinearExtension s_Climber
)
{
  private static boolean bound = false;

  public void bind()
  {
    if (!bound) 
    {
      bindState();
      bindDrive();
      bindShooters();
      bindIntake();
      bindClimber();
      bound = true;
    }
  }

  private void bindState()
  {
    // Auto Aim
    switchboard.button(IDConstants.autoAimSwitchID)
      .onChange(runOnce(() -> PBDash.IO_AUTO_AIM.put(switchboard.button(IDConstants.autoAimSwitchID).getAsBoolean())).ignoringDisable(true));

    // Auto Pass
    switchboard.button(IDConstants.autoPassSwitchID)
      .onChange(runOnce(() -> PBDash.IO_AUTO_PASS.put(switchboard.button(IDConstants.autoPassSwitchID).getAsBoolean())).ignoringDisable(true));

    // Auto Rev
    switchboard.button(IDConstants.autoRevSwitchID)
      .onChange(runOnce(() -> PBDash.IO_AUTO_REV.put(switchboard.button(IDConstants.autoRevSwitchID).getAsBoolean())).ignoringDisable(true));

    // Fencing
    switchboard.button(IDConstants.fencingSwitchID)
      .onChange(runOnce(() -> PBDash.IO_FENCE.put(switchboard.button(IDConstants.fencingSwitchID).getAsBoolean())).ignoringDisable(true));

    // Vision
    switchboard.button(IDConstants.visionSwitchID)
      .onChange(runOnce(() -> PBDash.IO_LL.put(switchboard.button(IDConstants.visionSwitchID).getAsBoolean())).ignoringDisable(true));

    // Nudging
    driver.b().onTrue(runOnce(() -> state.nudging = false).ignoringDisable(true));
    driver.a().onTrue(runOnce(() -> state.nudging = true).ignoringDisable(true));
  }

  private void bindDrive()
  {
    // Heading reset
    driver.start()
      .onTrue(runOnce(() -> s_Swerve.resetRotation(Rotation2d.kZero)));

    s_Swerve.setDefaultCommand(DriveBuilder.manual());

    // Bump state.nudging
    bumpTrigger
      .and(PBDash.IO_FENCE::get)
      .and(s_Vision::hasLocalisation)
      .and(() -> state.nudging)
      .onTrue(s_Intake.bumpSafeCommand())
      .whileTrue(DriveBuilder.nonCardinal(bumpRotationTolerance))
      .onFalse(s_Intake.deployCommand());
    
    // Trench state.nudging
    trenchTrigger
      .and(PBDash.IO_FENCE::get)
      .and(s_Vision::hasLocalisation)
      .and(() -> state.nudging)
      .whileTrue(DriveBuilder.trenchNudge());

    // Climb heading lock
    driver.x()
      .and(() -> state.climbPos != ClimbPosition.None)
      .onTrue
      (
        DriveBuilder.headingLocked
        (
          () -> {
            var rotation = switch (state.climbPos) 
            {
              case Left -> Rotation2d.kCCW_90deg;
              case Right -> Rotation2d.kCW_90deg;
              case None -> Rotation2d.kZero; // Shouldn't actually happen due to trigger condition
            };
            return FieldUtils.allianceRotateRotation(rotation);
          }
        )
      );

    // Update throttle limits
    PBDash.IO_MAX_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMaxThrottle(PBDash.IO_MAX_THROTTLE.get())));
    PBDash.IO_MIN_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMinThrottle(PBDash.IO_MIN_THROTTLE.get())));
  }

  private void bindShooters()
  {
    final Trigger autoAimTrigger = PBDash.IO_AUTO_AIM.asTrigger().and(s_Vision::hasLocalisation);
    final Trigger allianceZoneTrigger = new Trigger(() -> FieldUtils.inAllianceZone(swerveStateSup.get().Pose.getTranslation()));

    PBDash.IO_AUTO_AIM.asTrigger()
      .and(() -> !s_Vision.hasLocalisation())
      .onTrue
      (
        runOnce(() -> {
          s_PortShooter.target.azimuth = -60;
          s_StbdShooter.target.azimuth = 60;
        })
      );

    // Targetting States
    autoAimTrigger
      .onFalse(bothShooters(s -> s.target.state = TargetState.Manual).ignoringDisable(true));
    autoAimTrigger
      .and(allianceZoneTrigger.negate())
      .onTrue(bothShooters(s -> s.target.state = TargetState.Point).ignoringDisable(true));
    autoAimTrigger
      .and(allianceZoneTrigger)
      .onTrue(bothShooters(s -> s.target.state = TargetState.Hub).ignoringDisable(true));

    // Pass Point
    autoAimTrigger.and(PBDash.IO_AUTO_PASS::get)
      .whileTrue(bothShooters(s -> s.target.point = FieldUtils.getClosestPassPoint(swerveStateSup.get().Pose.getTranslation())));
    
    // Revving/Idleing as Appropriate
    final Trigger shootActiveTrigger = driver.rightBumper().negate();
    final Trigger hubActiveTrigger = new Trigger(() -> FieldUtils.hubActiveToleranced(FieldUtils.getAlliance(), ControlConstants.preShiftShootMargin, ControlConstants.postShiftShootMargin));

    // Rev if auto aiming, auto revving, and shooters are active
    autoAimTrigger
      .and(PBDash.IO_AUTO_REV::get)
      .and(shootActiveTrigger)
      .and(allianceZoneTrigger.and(hubActiveTrigger).or(PBDash.IO_AUTO_PASS::get))
      .onTrue(bothShooters(Shooter::revFlywheels).ignoringDisable(true))
      .onFalse(bothShooters(Shooter::idleFlywheels).ignoringDisable(true));

    // Shooting when Ready
    shootActiveTrigger
      .and(s_PortShooter::shootReady)
      .whileTrue(s_PortShooter.runIndexerCommand());
    shootActiveTrigger
      .and(s_StbdShooter::shootReady)
      .whileTrue(s_StbdShooter.runIndexerCommand());
  }

  private void bindIntake()
  {
    // Off
    driver.leftBumper().onTrue(s_Intake.stopIntakeCommand());
    // On
    driver.rightBumper().onTrue(s_Intake.startIntakeCommand());

    // Deploy
    operator.leftBumper().onTrue(s_Intake.deployCommand());
    // Stow
    operator.rightBumper().onTrue(s_Intake.stowCommand());

    // Reverse
    operator.leftTrigger(ControlConstants.triggerThreshold)
      .whileTrue
      (
        new Command() 
        {
          private RollerState prevState;

          public void initialize()
          {
            prevState = s_Intake.state;
            s_Intake.state = RollerState.Reversed;
          }

          public void end(boolean i) {s_Intake.state = prevState;}
        }
      );
  }

  private void bindClimber()
  {
    // Set Climb
    driver.povLeft().onTrue(runOnce(() -> state.climbPos = ClimbPosition.Left));
    driver.povRight().onTrue(runOnce(() -> state.climbPos = ClimbPosition.Right));

    // Retract
    operator.start().onTrue(s_Climber.retractCommand());
    // Extend
    operator.back().onTrue(s_Climber.extendCommand());
      
    // Manual Control
    s_Climber.setDefaultCommand(s_Climber.adjustTargetCommand(() -> operator.getRightY() * ControlConstants.manualClimberExtensionScale));
  }

  /** Mutually exclusive to {@link Controls#bind bind()} */
  public void bindSysId()
  {
    if (!bound)
    {
      s_Swerve.setDefaultCommand(DriveBuilder.manual());

      driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
      driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

      /*
      * Joystick Y = quasistatic forward
      * Joystick A = quasistatic reverse
      * Joystick B = dynamic forward
      * Joystick X = dyanmic reverse
      */
      driver.y().whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      driver.a().whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driver.b().whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
      driver.x().whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      bound = true;
    }
  }

  private Command bothShooters(Consumer<Shooter> action)
  {
    return runOnce(() -> {
      action.accept(s_PortShooter);
      action.accept(s_StbdShooter);
    });
  }

  private Command bothShootersCmd(Function<Shooter, Command> cmd)
  {
    return parallel
    (
      cmd.apply(s_PortShooter),
      cmd.apply(s_StbdShooter)
    );
  }
}
