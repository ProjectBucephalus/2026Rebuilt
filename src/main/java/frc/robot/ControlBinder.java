package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.FieldConstants.GeoFencing.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.IntakeConstants.ExtensionConstants;
import frc.robot.constants.FieldConstants.FieldTuning;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.util.Conversions;
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
  //Supplier<Pose2d> poseSup,
  CommandXboxController driver,
  CommandXboxController operator,
  CommandGenericHID switchboard,
  JoystickTransmuter driverStick,
  Brake driverBrake,
  CommandSwerveDrivetrain s_Swerve,
  Vision s_Vision,
  Limelight s_PhotonPort,
  Limelight s_PhotonStbd,
  Shooter s_PortShooter,
  Shooter s_StbdShooter,
  Intake s_Intake,
  PositionMotor s_Extension,
  LinearExtension s_Climber,
  DigitalInput io_ClimberPost
)
{
  private static AtomicBoolean bound = new AtomicBoolean(false);
  private static Trigger switchboardConnected;
  private static Trigger shootZoneTrigger;
    
  public void bind()
  {
    if (bound.compareAndExchange(false, true)) return; // Guard against being called multiple times

    switchboardConnected = switchboard.button(1).or(switchboard.button(2)).or(switchboard.button(3));

    bindState();
    bindDrive();
    bindShooters();
    bindIntake();
    bindClimber();
  }

  private void bindState()
  {
    // Auto Score
    switchboard.button(IDConstants.shootHubSwitchID)
      .onChange(runOnce(() -> PBDash.IO_SHOOT_HUB.put(switchboard.button(IDConstants.shootHubSwitchID).getAsBoolean())).onlyIf(switchboardConnected).ignoringDisable(true));

    // Auto Pass
    switchboard.button(IDConstants.shootPassSwitchID)
      .onChange(runOnce(() -> PBDash.IO_SHOOT_PASS.put(switchboard.button(IDConstants.shootPassSwitchID).getAsBoolean())).onlyIf(switchboardConnected).ignoringDisable(true));

    // Fencing
    switchboard.button(IDConstants.fencingSwitchID)
      .onChange(runOnce(() -> PBDash.IO_FENCE.put(switchboard.button(IDConstants.fencingSwitchID).getAsBoolean())).onlyIf(switchboardConnected).ignoringDisable(true));

    // Vision
    switchboard.button(IDConstants.visionSwitchID)
      .onChange(runOnce(() -> PBDash.IO_LL.put(switchboard.button(IDConstants.visionSwitchID).getAsBoolean())).onlyIf(switchboardConnected).ignoringDisable(true));

    // Climber Wiggle
    switchboard.button(IDConstants.climbWiggleSwitchID).negate()
      .and(switchboardConnected)
      .onChange(runOnce(() -> PBDash.IO_CLIMB_WIGGLE.put(switchboard.button(IDConstants.climbWiggleSwitchID).getAsBoolean())).ignoringDisable(true));

    // Power Save
    new Trigger(PBDash.IO_POWER_DRIVE::get)
      .onTrue
      (runOnce(() -> {
        PBDash.IO_MAX_THROTTLE.put(ControlConstants.powerSaveThrottle);
        PBDash.IO_INTAKE_SPEED.put(IntakeConstants.RollerConstants.intakeMinSpeed);
      }))
      .onFalse
      (runOnce(() -> {
        PBDash.IO_MAX_THROTTLE.put(ControlConstants.maxThrottle);
        PBDash.IO_INTAKE_SPEED.put(IntakeConstants.RollerConstants.intakeMaxSpeed);
      }));
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
      .onFalse(s_Extension.setTargetCmd(() -> s_Extension.getAngle() > ExtensionConstants.bumpSafeRotations * 1.2 ? ExtensionConstants.maxRotations : ExtensionConstants.minRotations));
    
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
          () -> switch (state.climbPos) 
          {
            case Left -> Rotation2d.kCCW_90deg;
            case Right -> Rotation2d.kCW_90deg;
            case None -> Rotation2d.kZero; // Shouldn't actually happen due to trigger condition
          }
        ).onlyWhile(() -> state.climbPos != ClimbPosition.None)
      );

    // Update throttle limits
    PBDash.IO_MAX_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMaxThrottle(PBDash.IO_MAX_THROTTLE.get())));
    PBDash.IO_MIN_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMinThrottle(PBDash.IO_MIN_THROTTLE.get())));

    
    GeoFencing.climbBlueRight.asTrigger()
      .whileTrue(climbSequenceCmd(Alliance.Blue, ClimbPosition.Right));
    GeoFencing.climbBlueLeft.asTrigger()
      .whileTrue(climbSequenceCmd(Alliance.Blue, ClimbPosition.Left));
    GeoFencing.climbRedRight.asTrigger()
      .whileTrue(climbSequenceCmd(Alliance.Red, ClimbPosition.Right));
    GeoFencing.climbRedLeft.asTrigger()
      .whileTrue(climbSequenceCmd(Alliance.Red, ClimbPosition.Left));
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
        bothShooters(Commands::runOnce, s -> s.target.state = TargetState.Vision)
        .andThen
        (
          repeatingSequence
          (
            Commands.waitSeconds(0.1),
            runOnce(() -> {          
              s_PortShooter.target.azimuth -= 3;
              s_StbdShooter.target.azimuth += 3;
            })
          )
        ).until(s_Vision::hasLocalisation).withTimeout(3)
      );

    // Tag-Seeking for climb
    driver.povRight().or(() -> state.climbPos == ClimbPosition.Right && DriverStation.isAutonomous())
      .onTrue
      (
        runOnce(() -> {
          PBDash.DEVICE_ERRORS.put("Climb Right Vision");
          s_StbdShooter.target.state = TargetState.Vision;
          s_PhotonPort.setActive(false);
        })
        .andThen
        (
          waitSeconds(0.1),
          runOnce(() -> PBDash.DEVICE_ERRORS.append(" - Active")),
          runOnce(() -> s_StbdShooter.target.azimuth = 45)
        )
      );
    new Trigger(() -> state.climbPos != ClimbPosition.Right)
      .onTrue
      (
        runOnce(() -> {
          s_StbdShooter.target.state = TargetState.Hub;
          s_PhotonPort.setActive(true);
        })
      );

    driver.povLeft().or(() -> state.climbPos == ClimbPosition.Left && DriverStation.isAutonomous())
      .onTrue
      (
        runOnce(() -> {   
          PBDash.DEVICE_ERRORS.put("Climb Left Vision");
          s_PortShooter.target.state = TargetState.Vision;
          s_PhotonStbd.setActive(false);
        })
        .andThen
        (
          waitSeconds(0.1),
          runOnce(() -> PBDash.DEVICE_ERRORS.append(" - Active")),
          runOnce(() -> s_PortShooter.target.azimuth = -50)
        )
      );
    new Trigger(() -> state.climbPos != ClimbPosition.Left)
    .onTrue
      (
        runOnce(() -> {
          s_PortShooter.target.state = TargetState.Hub;
          s_PhotonStbd.setActive(true);
        })
      );

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
      .onTrue
      (
        bothShooters(Commands::runOnce, s -> s.target.state = TargetState.Hub)
          .onlyIf(s_Vision::hasLocalisation)
          .ignoringDisable(true)
      );

    switchboard.button(IDConstants.disableShootersSwitchID)
      .and(() -> state.shoot == ShootersState.Test)
      .onTrue
      (
        bothShooters(Commands::runOnce, s -> s.target.disabled = true)
          .ignoringDisable(true)
      )
      .onFalse
      (
        bothShooters(Commands::runOnce, s -> s.target.disabled = false)
          .onlyIf(switchboardConnected)
          .ignoringDisable(true)
      );

    switchboard.button(IDConstants.calibrateButtonID)
      .onTrue(bothShooters(Commands::runOnce, Shooter::calibrate).ignoringDisable(true));
    
    final Trigger autoAimTrigger = new Trigger(() -> state.shoot != ShootersState.Manual && state.shoot != ShootersState.Test)
                                          .and(s_Vision::hasLocalisation)
                                          .and(DriverStation::isEnabled);
    final Trigger allianceZoneTrigger = new Trigger(() -> FieldUtils.inAllianceZone(state.swerve.Pose.getTranslation()));

    // Not Manual, Not Auto, Outside Alliance Zone
    autoAimTrigger
      .and(allianceZoneTrigger.negate())
      .and(() -> !DriverStation.isAutonomous())
      .onTrue(bothShooters(Commands::runOnce, s -> s.target.state = TargetState.Point).ignoringDisable(true))
      .whileTrue(bothShooters(Commands::run, s -> s.target.point = FieldUtils.getPassPoint(state.swerve.Pose.getTranslation())));

    autoAimTrigger
      .and(allianceZoneTrigger.negate())
      .and(() -> DriverStation.isAutonomous())
      .onTrue(bothShooters(Commands::runOnce, s -> s.target.state = TargetState.Vision));

    // Not Manual, Inside Alliance Zone
    autoAimTrigger
      .and(allianceZoneTrigger)
      .onTrue(bothShooters(Commands::runOnce, s -> s.target.state = TargetState.Hub).ignoringDisable(true));

    // Port-Only
    new Trigger(() -> state.shoot == ShootersState.Port)
      .onTrue(s_StbdShooter.runOnce(() -> s_StbdShooter.target.disabled = true).ignoringDisable(true))
      .onFalse(s_StbdShooter.runOnce(() -> s_StbdShooter.target.disabled = false).ignoringDisable(true))
      .whileTrue(s_StbdShooter.runIndexerCmd(() -> -s_PortShooter.getSpeed()).onlyIf(() -> s_Extension.getAngle() > -0.2)); // Follow opposing indexer while shooter is disabled
    
    // Stbd-Only
    new Trigger(() -> state.shoot == ShootersState.Stbd)
      .onTrue(s_PortShooter.runOnce(() -> s_PortShooter.target.disabled = true).ignoringDisable(true))
      .onFalse(s_PortShooter.runOnce(() -> s_PortShooter.target.disabled = false).ignoringDisable(true))
      .whileTrue(s_PortShooter.runIndexerCmd(() -> -s_StbdShooter.getSpeed()).onlyIf(() -> s_Extension.getAngle() > -0.2)); // Follow opposing indexer while shooter is disabled

    // Rev If (test and fire) or (((not alliance_zone) or shift) and not test)
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
      .onTrue(bothShooters(Commands::runOnce, Shooter::revFlywheels))
      .onFalse(bothShooters(Commands::runOnce, Shooter::idleFlywheels).ignoringDisable(true));

    /* Shooting when Ready */

    // (alliance_zone and auto_hub) or ((not alliance_zone) and auto_pass)
    shootZoneTrigger = new Trigger(DriverStation::isEnabled)
      .and(allianceZoneTrigger.and(PBDash.IO_SHOOT_HUB.asTrigger()))
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
    .and(DriverStation::isEnabled)
    .whileTrue(s_PortShooter.runIndexerCmd().onlyIf(() -> s_Extension.getAngle() > -0.2));

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
    .and(DriverStation::isEnabled)
    .whileTrue(s_StbdShooter.runIndexerCmd().onlyIf(() -> s_Extension.getAngle() > -0.2));

    shootZoneTrigger.negate()
      .and(manualFireTrigger.negate())
      .and(DriverStation::isEnabled)
      .onTrue(bothShooters(Commands::runOnce, s -> s.target.flywheelsActive = false))
      .onFalse(bothShooters(Commands::runOnce, s -> s.target.flywheelsActive = true));
    
    // Shoot while climbing
    new Trigger(() -> s_Climber.getTarget() == ClimberConstants.climbPosition)
      .onTrue(runOnce
        (() -> {
          if (state.climbPos == ClimbPosition.Left)
          {
            s_StbdShooter.target.state = TargetState.Manual;
            s_StbdShooter.target.azimuth = ShooterConstants.towerAimStbdAz;
            s_StbdShooter.setDistance(ShooterConstants.towerAimStbdDist);
          }
          if (state.climbPos == ClimbPosition.Right)
          {
            s_PortShooter.target.state = TargetState.Manual;
            s_PortShooter.target.azimuth = ShooterConstants.towerAimPortAz;
            s_PortShooter.setDistance(ShooterConstants.towerAimPortDist);
          }
        })
      );
  }

  private void bindIntake()
  {
    // Off
    driver.rightBumper().onTrue(s_Intake.setStateCmd(RollerState.Off));
    // On
    driver.leftBumper()
      .onTrue(s_Intake.setStateCmd(RollerState.On))
      .onFalse(s_Intake.setStateCmd(RollerState.Idle));

    // Deploy
    operator.leftBumper().or(driver.leftBumper())
      .onTrue(s_Extension.setTargetCmd(() -> ExtensionConstants.maxRotations))
      .onTrue(runOnce(() -> PBDash.EXTENSION_STATE.put("Deployed")));
    // Stow
    operator.rightBumper()
      .and(operator.rightTrigger().negate())
      .onTrue(s_Extension.setTargetCmd(() -> ExtensionConstants.minRotations))
      .onTrue(runOnce(() -> PBDash.EXTENSION_STATE.put("Stowed")));
    // Jostle
    operator.rightTrigger()
      .and(operator.rightBumper().or(shootZoneTrigger))
      .and(() -> s_Extension.getTarget() != ExtensionConstants.minRotations)
      .onTrue(s_Extension.setTargetCmd(() -> ExtensionConstants.jostleRotations)
        .alongWith(
          runOnce(() -> PBDash.EXTENSION_STATE.put("Jostle")),
          s_Intake.setStateCmd(RollerState.On)))
      .onFalse(s_Extension.setTargetCmd(() -> ExtensionConstants.maxRotations)
        .alongWith(
          runOnce(() -> PBDash.EXTENSION_STATE.put("Deployed")),
          s_Intake.setStateCmd(RollerState.Idle)));

    // Reverse
    operator.leftTrigger(ControlConstants.triggerThreshold)
      .whileTrue
      (
        new Command() 
        {
          {addRequirements(s_Extension);}
          
          private RollerState prevState;

          @Override
          public void initialize()
          {
            prevState = s_Intake.state;
            s_Intake.state = RollerState.Reversed;
          }

          @Override
          public void end(boolean i) 
          {
            s_Intake.state = prevState;
          }
        }
      );

    // Manual Control
    s_Extension.setDefaultCommand(s_Extension.adjustTargetCmd(() -> MathUtil.applyDeadband(operator.getLeftY(), ControlConstants.manualControlDeadband) * ControlConstants.manualIntakeExtensionScale));
    operator
      .axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, ControlConstants.manualControlDeadband)
      .onTrue(runOnce(() -> PBDash.EXTENSION_STATE.put("Manual")));
  }

  private void bindClimber()
  {
    final Trigger autoDeployTrigger = new Trigger(() -> state.climbPos != ClimbPosition.None);
    final Trigger allianceZoneTrigger = new Trigger(() -> FieldUtils.inAllianceZone(state.swerve.Pose.getTranslation()));

    // In alliance zone and auto-deploy, extend (only on true so that manual control can still happen while in alliance zone)
    autoDeployTrigger
      .onTrue(s_Climber.extendCmd().alongWith(Commands.runOnce(() -> PBDash.CLIMBER_STATE.put("Extended")))
        .onlyIf(allianceZoneTrigger.and(s_Vision::hasLocalisation)));

    // Leave alliance zone or enter trench, retract (intentionally regardless of auto-deploy)
    trenchTrigger
      .or(allianceZoneTrigger.negate())
      .and(s_Vision::hasLocalisation)
      .onTrue(s_Climber.retractCmd().alongWith(Commands.runOnce(() -> PBDash.CLIMBER_STATE.put("Home"))));

    // Set Climb
    driver.povLeft().onTrue(runOnce(() -> state.climbPos = ClimbPosition.Left));
    driver.povRight().onTrue(runOnce(() -> state.climbPos = ClimbPosition.Right));
    driver.back().onTrue(runOnce(() -> state.climbPos = ClimbPosition.None)); // TODO: Rebind to povUp?

    // Retract
    operator.start()
      .or(switchboard.button(IDConstants.climbButtonID))
      .onTrue
      (
        Commands.either
        (
          s_Climber.setTargetCmd(ClimberConstants.climbPosition)
            .alongWith(runOnce(() -> {PBDash.CLIMBER_STATE.put("Climb");})), 
          s_Climber.retractCmd()
            .alongWith(runOnce(() -> PBDash.CLIMBER_STATE.put("Home"))), 
          () -> s_Climber.atMax() && !io_ClimberPost.get()
        )
      );
    // Extend
    operator.back()
      .onTrue(s_Climber.extendCmd())
      .onTrue(runOnce(() -> PBDash.CLIMBER_STATE.put("Extended")));
      
    // Manual Control
    s_Climber.setDefaultCommand(s_Climber.adjustTargetCmd(() -> MathUtil.applyDeadband(-operator.getRightY(), ControlConstants.manualControlDeadband) * ControlConstants.manualClimberExtensionScale));
    operator
      .axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, ControlConstants.manualControlDeadband)
      .onTrue(runOnce(() -> PBDash.CLIMBER_STATE.put("Manual")));

    // Wiggle Test
    operator.rightStick()
      .whileTrue
      (
        Commands.repeatingSequence
        ( 
          s_Climber.gotoTargetCmd(ClimberConstants.maxPosition - ClimberConstants.wiggleOffset),
          Commands.waitSeconds(ClimberConstants.wiggleWait),
          s_Climber.gotoTargetCmd(ClimberConstants.maxPosition),
          Commands.waitSeconds(ClimberConstants.wiggleWait)
        )
      );
  }

  /** Mutually exclusive to {@link ControlBinder#bind bind()} */
  public void bindSysId()
  {
    if (bound.compareAndExchange(false, true)) return; // Guard against being called multiple times

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

  private Command climbSequenceCmd(Alliance alliance, ClimbPosition side)
  {

    Supplier<Command> approachPath; 
    Supplier<Command> climbPath;
    double maxHeight;

    if (side == ClimbPosition.Left)
      if(alliance == Alliance.Blue)
      {
        approachPath = () -> 
            DriveBuilder.pathFollow(Path.climbApproachBlueLeft.allianceOffset(PBDash.TUNE_CLIMB_BL.get(), 0))
            .until(() -> Conversions.nearTranslation(state.swerve.Pose.getTranslation(), Path.climbBlueLeft.targetPose().getTranslation(), 0.35));
        climbPath =() -> DriveBuilder.pathFollow(Path.climbBlueLeft.allianceOffset(PBDash.TUNE_CLIMB_BL.get(), 0));
        maxHeight = FieldTuning.postHeightOffsetBlueLeft;
      }
      else // if Alliance.Red
      {
        approachPath = () -> 
            DriveBuilder.pathFollow(Path.climbApproachRedLeft.allianceOffset(PBDash.TUNE_CLIMB_RL.get(), 0))
            .until(() -> Conversions.nearTranslation(state.swerve.Pose.getTranslation(), Path.climbRedLeft.targetPose().getTranslation(), 0.35));
        climbPath = () -> DriveBuilder.pathFollow(Path.climbRedLeft.allianceOffset(PBDash.TUNE_CLIMB_RL.get(), 0));
        maxHeight = FieldTuning.postHeightOffsetRedLeft;
      }
    else // if Right
      if(alliance == Alliance.Blue)
      {
        approachPath = () -> 
            DriveBuilder.pathFollow(Path.climbApproachBlueRight.allianceOffset(PBDash.TUNE_CLIMB_BR.get(), 0))
            .until(() -> Conversions.nearTranslation(state.swerve.Pose.getTranslation(), Path.climbBlueRight.targetPose().getTranslation(), 0.35));
        climbPath = () -> DriveBuilder.pathFollow(Path.climbBlueRight.allianceOffset(PBDash.TUNE_CLIMB_BR.get(), 0));
        maxHeight = FieldTuning.postHeightOffsetBlueRight;
      }
      else // if Alliance.Red
      {
        approachPath = () -> 
            DriveBuilder.pathFollow(Path.climbApproachRedRight.allianceOffset(PBDash.TUNE_CLIMB_RR.get(), 0))
            .until(() -> Conversions.nearTranslation(state.swerve.Pose.getTranslation(), Path.climbRedRight.targetPose().getTranslation(), 0.35));
        climbPath = () -> DriveBuilder.pathFollow(Path.climbRedRight.allianceOffset(PBDash.TUNE_CLIMB_RR.get(), 0));
        maxHeight = FieldTuning.postHeightOffsetRedRight;
      }

    return s_Climber.extendCmd()
      .andThen
      (
        // Extend climber and do initial approach, make sure climber is extended
        Commands.runOnce(() -> PBDash.CLIMBER_STATE.put("Extended")),
        s_Swerve.defer(approachPath),
        Commands.waitUntil(io_ClimberPost::get),

        Commands.either
        (
          // Wiggle climber on final approach to climb
          Commands.repeatingSequence
          ( 
            s_Climber.gotoTargetCmd(maxHeight - ClimberConstants.wiggleOffset),
            Commands.waitSeconds(ClimberConstants.wiggleWait),
            s_Climber.gotoTargetCmd(maxHeight),
            Commands.waitSeconds(ClimberConstants.wiggleWait)
          )
          .raceWith(s_Swerve.defer(climbPath)),

          // Or just do final approach
          s_Swerve.defer(climbPath),

          // Depending on switch state
          PBDash.IO_CLIMB_WIGGLE::get
        ),

        // Wait for further instruction
        s_Climber.extendCmd(),
        DriveBuilder.waitCommand()
      );
  }
}
