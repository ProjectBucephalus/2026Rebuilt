package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.FieldConstants.GeoFencing.*;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot.ClimbPosition;
import frc.robot.Robot.RobotState;
import frc.robot.Robot.ShootersState;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Path;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.IntakeConstants.ExtensionConstants;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;
import frc.robot.controlTransmutation.Brake;
import frc.robot.controlTransmutation.JoystickTransmuter;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.RollerState;
import frc.robot.subsystems.generic.LinearExtension;
import frc.robot.subsystems.generic.PositionMotor;

public record ControlBinder
(
  RobotState state,
  Supplier<Pose2d> poseSup,
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
  PositionMotor s_Extension,
  LinearExtension s_Climber,
  DigitalInput io_ClimberPost
)
{
  private static boolean bound = false;

  public void bind()
  {
    if (bound) return; // Guard against being called multiple times
    bound = true;

    bindState();
    bindDrive();
    bindShooters();
    bindIntake();
    bindClimber();
  }

  private void bindState()
  {
    // Auto Pass
    switchboard.button(IDConstants.shootHubSwitchID)
      .onChange(runOnce(() -> PBDash.IO_SHOOT_HUB.put(switchboard.button(IDConstants.shootHubSwitchID).getAsBoolean())).ignoringDisable(true));

    // Auto Rev
    switchboard.button(IDConstants.shootPassSwitchID)
      .onChange(runOnce(() -> PBDash.IO_SHOOT_PASS.put(switchboard.button(IDConstants.shootPassSwitchID).getAsBoolean())).ignoringDisable(true));

    // Fencing
    switchboard.button(IDConstants.fencingSwitchID)
      .onChange(runOnce(() -> PBDash.IO_FENCE.put(switchboard.button(IDConstants.fencingSwitchID).getAsBoolean())).ignoringDisable(true));

    // Vision
    switchboard.button(IDConstants.visionSwitchID)
      .onChange(runOnce(() -> PBDash.IO_LL.put(switchboard.button(IDConstants.visionSwitchID).getAsBoolean())).ignoringDisable(true));
  }

  private void bindDrive()
  {
    // Nudging
    driver.b().onTrue(runOnce(() -> state.nudging = false).ignoringDisable(true));
    driver.a().onTrue(runOnce(() -> state.nudging = true).ignoringDisable(true));

    // Heading reset
    driver.start()
      .onTrue(runOnce(() -> s_Swerve.resetRotation(Rotation2d.kZero)).ignoringDisable(true));

    s_Swerve.setDefaultCommand(DriveBuilder.manual());

    // Bump nudging
    bumpTrigger
      .and(() -> state.nudging)
      .onTrue(s_Extension.setTargetCmd(() -> Math.min(ExtensionConstants.bumpSafeRotations, s_Extension.getAngle())))
      .whileTrue(DriveBuilder.nonCardinal(bumpRotationTolerance))
      .onFalse(s_Extension.setTargetCmd(() -> ExtensionConstants.maxRotations));
    
    // Trench nudging
    trenchTrigger
      .and(() -> state.nudging)
      .whileTrue(DriveBuilder.trenchNudge());

    // Unlock heading
    driver.axisMagnitudeGreaterThan(XboxController.Axis.kRightX.value, ControlConstants.stickDeadband)
      .onTrue(s_Swerve.getDefaultCommand());

    // Climb heading lock
    driver.x()
      .and(() -> state.climbPos != ClimbPosition.None)
      .or(driver.povLeft())
      .or(driver.povRight())
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
            return rotation;
          }
        ).onlyWhile(() -> state.climbPos != ClimbPosition.None)
      );

    // Update throttle limits
    PBDash.IO_MAX_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMaxThrottle(PBDash.IO_MAX_THROTTLE.get())));
    PBDash.IO_MIN_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMinThrottle(PBDash.IO_MIN_THROTTLE.get())));

    GeoFencing.climbBlueRight.asTrigger()
      .whileTrue(DriveBuilder.pathFollow(Path.climbBlueRight).andThen(DriveBuilder.waitCommand()));
    GeoFencing.climbBlueLeft.asTrigger()
      .whileTrue(DriveBuilder.pathFollow(Path.climbBlueLeft).andThen(DriveBuilder.waitCommand()));
    GeoFencing.climbRedRight.asTrigger()
      .whileTrue(DriveBuilder.pathFollow(Path.climbRedRight).andThen(DriveBuilder.waitCommand()));
    GeoFencing.climbRedLeft.asTrigger()
      .whileTrue(DriveBuilder.pathFollow(Path.climbRedLeft).andThen(DriveBuilder.waitCommand()));
  }

  private void bindShooters()
  {
    // Change shooter state
    operator.a().onTrue(runOnce(() -> state.shoot = ShootersState.Auto));
    operator.b().onTrue(runOnce(() -> state.shoot = ShootersState.Stbd));
    operator.x().onTrue(runOnce(() -> state.shoot = ShootersState.Port));
    operator.y().onTrue(runOnce(() -> state.shoot = ShootersState.Manual));

    // Set manual shooting distance
    operator.povUp()
      .and(() -> state.shoot == ShootersState.Manual)
      .onTrue(bothShooters(Commands::runOnce, s -> s.setDistance(ShooterConstants.closeManualRange)));
    operator.povDown()
      .and(() -> state.shoot == ShootersState.Manual)
      .onTrue(bothShooters(Commands::runOnce, s -> s.setDistance(ShooterConstants.farManualRange)));

    final Trigger manualFireTrigger = operator.rightTrigger(ControlConstants.triggerThreshold);

    // Tag-Seeking if no Localisation
    new Trigger(() -> state.shoot != ShootersState.Manual && state.shoot != ShootersState.Test)
      .and(manualFireTrigger.negate())
      .and(() -> !s_Vision.hasLocalisation())
      .and(PBDash.IO_LL::get)
      .onTrue
      (
        runOnce(() -> {          
          s_PortShooter.target.azimuth = -60;
          s_StbdShooter.target.azimuth = 60;
        })
        .alongWith(bothShooters(Commands::runOnce, s -> s.target.state = TargetState.Manual))
        .ignoringDisable(true)
      );

    // Tag-Seeking for climb
    new Trigger(() -> state.climbPos == ClimbPosition.Right)
      .onTrue
      (
        runOnce(() -> {          
          s_StbdShooter.target.azimuth = 45;
          s_StbdShooter.target.state = TargetState.Manual;
        })
      )
      .onFalse(runOnce(() -> s_StbdShooter.target.state = s_PortShooter.target.state));

    new Trigger(() -> state.climbPos == ClimbPosition.Left)
      .onTrue
      (
        runOnce(() -> {          
          s_PortShooter.target.azimuth = -45;
          s_PortShooter.target.state = TargetState.Manual;
        })
      )
      .onFalse(runOnce(() -> s_PortShooter.target.state = s_StbdShooter.target.state));

    // Manual
    new Trigger(() -> state.shoot == ShootersState.Manual)
      .onTrue
      (
        bothShooters(Commands::runOnce, s -> {
          s.target.state = TargetState.Manual;
          s.target.azimuth = 0;
        }).ignoringDisable(true)
      );

    // Test (Using dashboard values)
    switchboard.button(IDConstants.testManualSwitchID)
      .and(() -> state.shoot == ShootersState.Test)
      .whileTrue
      (
        bothShooters(Commands::runOnce, s -> {
          s.target.state = TargetState.Manual;
          s.target.azimuth = PBDash.TEST_AZIMUTH.get();
          s.target.altitude = PBDash.TEST_ALTITUDE.get();
          s.target.speed = PBDash.TEST_FLYSPEED.get();
        })
        .repeatedly()
        .ignoringDisable(true)
      );

    switchboard.button(IDConstants.testHubSwitchID)
      .and(() -> state.shoot == ShootersState.Test)
      .whileTrue
      (
        bothShooters(Commands::runOnce, s -> {
          s.target.state = TargetState.Hub;
        })
        .repeatedly()
        .ignoringDisable(true)
      );

    switchboard.button(IDConstants.testIdleSwitchID)
      .and(() -> state.shoot == ShootersState.Test)
      .whileTrue
      (
        bothShooters(Commands::runOnce, s -> {
          s.target.state = TargetState.Manual;
        })
        .repeatedly()
        .ignoringDisable(true)
      );

    switchboard.button(10)
      .onTrue
      (
        bothShooters(Commands::runOnce, s -> {s.calibrate();})
        .ignoringDisable(true)
      );
    
    final Trigger autoAimTrigger = new Trigger(() -> state.shoot != ShootersState.Manual && state.shoot != ShootersState.Test)
                                          .and(s_Vision::hasLocalisation)
                                          .and(DriverStation::isEnabled);
    final Trigger allianceZoneTrigger = new Trigger(() -> FieldUtils.inAllianceZone(poseSup.get().getTranslation()));

    // Not Manual, Outside Alliance Zone
    autoAimTrigger
      .and(allianceZoneTrigger.negate())
      .onTrue(bothShooters(Commands::runOnce, s -> s.target.state = TargetState.Point).ignoringDisable(true))
      .whileTrue(bothShooters(Commands::run, s -> s.target.point = FieldUtils.getPassPoint(poseSup.get().getTranslation())));

    // Not Manual, Inside Alliance Zone
    autoAimTrigger
      .and(allianceZoneTrigger)
      .onTrue(bothShooters(Commands::runOnce, s -> s.target.state = TargetState.Hub).ignoringDisable(true));

    // Port-Only
    new Trigger(() -> state.shoot == ShootersState.Port)
      .onTrue(s_StbdShooter.runOnce(() -> s_StbdShooter.target.disabled = true).ignoringDisable(true))
      .onFalse(s_StbdShooter.runOnce(() -> s_StbdShooter.target.disabled = false).ignoringDisable(true))
      .whileTrue(s_StbdShooter.runIndexerCmd(() -> -s_PortShooter.getSpeed())); // Follow opposing indexer while shooter is disabled
    
    // Stbd-Only
    new Trigger(() -> state.shoot == ShootersState.Stbd)
      .onTrue(s_PortShooter.runOnce(() -> s_PortShooter.target.disabled = true).ignoringDisable(true))
      .onFalse(s_PortShooter.runOnce(() -> s_PortShooter.target.disabled = false).ignoringDisable(true))
      .whileTrue(s_PortShooter.runIndexerCmd(() -> -s_StbdShooter.getSpeed())); // Follow opposing indexer while shooter is disabled

    // Rev if (test and fire) or (((not alliance_zone) or shift) and not test)
    new Trigger
      (() ->
        (state.shoot == ShootersState.Test && operator.rightTrigger().getAsBoolean())
        ||
        (
          DriverStation.isEnabled()
          &&
          (!allianceZoneTrigger.getAsBoolean() || FieldUtils.hubActiveToleranced(ControlConstants.preShiftMargin, ControlConstants.postShiftMargin))
          && 
          state.shoot != ShootersState.Test
        )
      )
      .onTrue(bothShooters(Commands::runOnce, Shooter::revFlywheels).ignoringDisable(true))
      .onFalse(bothShooters(Commands::runOnce, Shooter::idleFlywheels).ignoringDisable(true));

    /* Shooting when Ready */

    // (alliance_zone and auto_hub) or ((not alliance_zone) and auto_pass)
    final Trigger shootZoneTrigger = 
         (allianceZoneTrigger.and(PBDash.IO_SHOOT_HUB.asTrigger()))
      .or(allianceZoneTrigger.negate().and(PBDash.IO_SHOOT_PASS.asTrigger()));

    final Trigger forceStopTrigger = driver.leftTrigger(ControlConstants.triggerThreshold);

    // Port
    forceStopTrigger.negate()
    .and
    (() ->
      s_PortShooter.shootReady()
      &&
      (
        (state.shoot != ShootersState.Stbd && manualFireTrigger.getAsBoolean())
        ||
        ((state.shoot == ShootersState.Auto || state.shoot == ShootersState.Port) && shootZoneTrigger.getAsBoolean())
      )
    )
    .whileTrue(s_PortShooter.runIndexerCmd());

    // Stbd
    forceStopTrigger.negate()
    .and
    (() ->
      s_StbdShooter.shootReady()
      &&
      (
        (state.shoot != ShootersState.Port && manualFireTrigger.getAsBoolean())
        ||
        ((state.shoot == ShootersState.Auto || state.shoot == ShootersState.Stbd) && shootZoneTrigger.getAsBoolean())
      )
    )
    .whileTrue(s_StbdShooter.runIndexerCmd());
  }

  private void bindIntake()
  {
    // Off
    driver.leftBumper().onTrue(s_Intake.setStateCmd(RollerState.Off));
    // On
    driver.rightBumper().onTrue(s_Intake.setStateCmd(RollerState.On));

    // Deploy
    operator.leftBumper().or(driver.rightBumper())
      .onTrue(s_Extension.setTargetCmd(() -> ExtensionConstants.maxRotations));
    // Stow
    operator.rightBumper().onTrue(s_Extension.setTargetCmd(() -> ExtensionConstants.minRotations));

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

    // Manual Control
    s_Extension.setDefaultCommand(s_Extension.adjustTargetCmd(() -> MathUtil.applyDeadband(operator.getLeftY(), ControlConstants.manualControlDeadband) * ControlConstants.manualIntakeExtensionScale));
  }

  private void bindClimber()
  {
    final Trigger autoDeployTrigger = new Trigger(() -> state.climbPos != ClimbPosition.None);
    final Trigger allianceZoneTrigger = new Trigger(() -> FieldUtils.inAllianceZone(poseSup.get().getTranslation()));

    // In alliance zone and auto-deploy, extend (only on true so that manual control can still happen while in alliance zone)
    autoDeployTrigger
      .and(allianceZoneTrigger)
      .and(s_Vision::hasLocalisation)
      .onTrue(s_Climber.extendCmd());

    // Leave alliance zone or enter trench, retract (intentionally regardless of auto-deploy)
    trenchTrigger
      .or(allianceZoneTrigger.negate())
      .and(s_Vision::hasLocalisation)
      .onTrue(s_Climber.retractCmd());

    // Set Climb
    driver.povLeft().onTrue(runOnce(() -> state.climbPos = ClimbPosition.Left));
    driver.povRight().onTrue(runOnce(() -> state.climbPos = ClimbPosition.Right));
    driver.back().onTrue(runOnce(() -> state.climbPos = ClimbPosition.None));

    // Retract
    operator.start()
      .onTrue
      (
        Commands.either
        (
          s_Climber.setTargetCmd(ClimberConstants.climbPosition), 
          s_Climber.retractCmd(), 
          () -> s_Climber.atMax() && !io_ClimberPost.get()
        )
      );
    // Extend
    operator.back().onTrue(s_Climber.extendCmd());
      
    // Manual Control
    s_Climber.setDefaultCommand(s_Climber.adjustTargetCmd(() -> MathUtil.applyDeadband(-operator.getRightY(), ControlConstants.manualControlDeadband) * ControlConstants.manualClimberExtensionScale));
  }

  /** Mutually exclusive to {@link ControlBinder#bind bind()} */
  public void bindSysId()
  {
    if (bound) return; // Guard against being called multiple times
    bound = true;

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
  }
 
  private Command bothShooters(Function<Runnable, Command> cmd, Consumer<Shooter> action)
  {
    return cmd.apply(() -> {
      action.accept(s_PortShooter);
      action.accept(s_StbdShooter);
    });
  }
}
