package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.FieldConstants.GeoFencing.*;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot.ButtonPadState;
import frc.robot.Robot.ClimbPosition;
import frc.robot.Robot.HeadingLockState;
import frc.robot.Robot.RobotState;
import frc.robot.commands.swerve.ClimbLockedDrive;
import frc.robot.commands.swerve.ManualDrive;
import frc.robot.commands.swerve.NonCardinalDrive;
import frc.robot.commands.swerve.OutpostLockedDrive;
import frc.robot.commands.swerve.TrenchLockedDrive;
import frc.robot.commands.swerve.TrenchNudgeDrive;
import frc.robot.constants.ButtonPadConstants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AlliancePose2d;
import frc.robot.util.AllianceTranslation2d;
import frc.robot.util.FieldUtils;
import frc.robot.util.Launchpad;
import frc.robot.util.Launchpad.PadColour;
import frc.robot.util.PBDash;
import frc.robot.util.controlTransmutation.Brake;
import frc.robot.util.controlTransmutation.JoystickTransmuter;
import frc.robot.subsystems.*;
import frc.robot.subsystems.generic.LinearExtension;

public class Controls 
{
  private static Shooter s_PortShooter;
  private static Shooter s_StbdShooter;

  public static void bind
  (
    RobotState state,
    Supplier<SwerveDriveState> swerveStateSup,
    CommandXboxController driver,
    CommandXboxController debug,
    Launchpad buttonPad,
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
    Controls.s_PortShooter = s_PortShooter;
    Controls.s_StbdShooter = s_StbdShooter;
    // -------------STATE--------------- //

    final Trigger autoAimTrigger = PBDash.IO_AUTO_AIM.asTrigger().and(s_Vision::hasLocalisation);
    final Trigger allianceZoneTrigger = new Trigger(() -> FieldUtils.inAllianceZone(swerveStateSup.get().Pose.getTranslation()));

    // Button pad modes
    final Trigger btnSetPass          = new Trigger(() -> state.btnSet == ButtonPadState.PassPointSelection);
    final Trigger btnSetLocalisation  = new Trigger(() -> state.btnSet == ButtonPadState.LocalisationOveride);
    final Trigger btnSetManual        = new Trigger(() -> state.btnSet == ButtonPadState.ManualControls);

    // Auto aim switch
    switchboard.button(IDConstants.autoAimSwitchID)
      .onChange(runOnce(() -> PBDash.IO_AUTO_AIM.put(switchboard.button(IDConstants.autoAimSwitchID).getAsBoolean())).ignoringDisable(true));
    debug.rightStick().onTrue(runOnce(() -> PBDash.IO_AUTO_AIM.put(false)).ignoringDisable(true));

    buttonPad.M1().onTrue(runOnce(() -> PBDash.IO_AUTO_AIM.put(true)).ignoringDisable(true));
    
    // Auto pass switch
    switchboard.button(IDConstants.autoPassSwitchID)
      .onChange(runOnce(() -> PBDash.IO_AUTO_PASS.put(switchboard.button(IDConstants.autoPassSwitchID).getAsBoolean())).ignoringDisable(true));
    
    // Auto rev switch
    switchboard.button(IDConstants.autoRevSwitchID)
      .onChange(runOnce(() -> PBDash.IO_AUTO_REV.put(switchboard.button(IDConstants.autoRevSwitchID).getAsBoolean())).ignoringDisable(true));

    // Geofence and Vision switches
    switchboard.button(IDConstants.fencingSwitchID)
      .onChange(runOnce(() -> PBDash.IO_FENCE.put(switchboard.button(IDConstants.fencingSwitchID).getAsBoolean())).ignoringDisable(true));
    switchboard.button(IDConstants.visionSwitchID)
      .onChange(runOnce(() -> PBDash.IO_LL.put(switchboard.button(IDConstants.visionSwitchID).getAsBoolean())).ignoringDisable(true));

    // state.nudging
    driver.back().onTrue(runOnce(() -> state.nudging = false).ignoringDisable(true));
    driver.y().or(driver.b()).onTrue(runOnce(() -> state.nudging = true).ignoringDisable(true));
    driver.back().or(driver.y()).or(driver.b()).onFalse(runOnce(() -> PBDash.STATE_NUDGING.put(state.nudging)).ignoringDisable(true));
    
    
    
    // -------------DRIVE--------------- //

    s_Swerve.setDefaultCommand
    (
      new ManualDrive
      (
        s_Swerve, 
        driverStick::stickOutput,
        () -> -driver.getRightX(),
        driver::getRightTriggerAxis
      )
    );

    // Heading reset when not using vision
    driver.start()
      .onTrue(runOnce(() -> s_Swerve.resetRotation(Rotation2d.kZero)));

    // Bump state.nudging
    PBDash.IO_FENCE.asTrigger()
      .and(() -> state.nudging && s_Vision.hasLocalisation())
      .and
      (
            bumpNB.asTrigger()
        .or(bumpSB.asTrigger())
        .or(bumpNR.asTrigger())
        .or(bumpSR.asTrigger())
      )
      .onTrue(s_Intake.bumpSafeCommand())
      .whileTrue
      (
        new NonCardinalDrive
        (
          s_Swerve, 
          driverStick::stickOutput, 
          () -> -driver.getRightX(), 
          driver::getRightTriggerAxis, 
          () -> swerveStateSup.get().Pose.getRotation(), 
          bumpRotationTolerance
        )
      );
    
    // Trench state.nudging
    PBDash.IO_FENCE.asTrigger()
      .and(trenchTrigger)
      .and(() -> state.nudging && s_Vision.hasLocalisation())
      .whileTrue
      (
        new TrenchNudgeDrive
        (
          s_Swerve, 
          driverStick::stickOutput, 
          () -> -driver.getRightX(), 
          driver::getRightTriggerAxis, 
          () -> swerveStateSup.get().Pose.getRotation()
        ).onlyIf(() -> state.headingLock == HeadingLockState.Unlocked)
      );

    // Reset to manual drive when heading unlocks
    new Trigger(() -> state.headingLock == HeadingLockState.Unlocked)
      .onTrue(s_Swerve.getDefaultCommand());

    // Unlock heading
    driver.axisMagnitudeGreaterThan(XboxController.Axis.kRightX.value, ControlConstants.stickDeadband)
      .onTrue(runOnce(() -> state.headingLock = HeadingLockState.Unlocked));

    // Lock heading
    driver.y()
      .or(driver.b())
      .or(driver.a())
      .onTrue(runOnce(() -> state.headingLock = HeadingLockState.General));

    // Bump heading lock
    driver.b().onTrue
    (        
      new NonCardinalDrive
      (
        s_Swerve, 
        driverStick::stickOutput, 
        () -> -driver.getRightX(), 
        driver::getRightTriggerAxis, 
        () -> swerveStateSup.get().Pose.getRotation(), 
        bumpRotationTolerance
      )
    );

    // Trench heading lock
    driver.y().onTrue(new TrenchLockedDrive(s_Swerve, driverStick::stickOutput, () -> swerveStateSup.get().Pose));

    // Climb heading lock
    driver.x().onTrue(new ClimbLockedDrive(s_Swerve, driverStick::stickOutput, () -> swerveStateSup.get().Pose, () -> state.climbPos));

    // Outpost heading lock
    driver.a().onTrue(new OutpostLockedDrive(s_Swerve, driverStick::stickOutput, () -> swerveStateSup.get().Pose));

    // Update throttle limits
    PBDash.IO_MAX_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMaxThrottle(PBDash.IO_MAX_THROTTLE.get())));
    PBDash.IO_MIN_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMinThrottle(PBDash.IO_MIN_THROTTLE.get())));


    // -------------SHOOTERS------------ //

    // Targetting States
    autoAimTrigger
      .onFalse(bothShooters(s -> s.getTarget().state = TargetState.Manual).ignoringDisable(true));
    autoAimTrigger
      .and(allianceZoneTrigger.negate())
      .onTrue(bothShooters(s -> s.getTarget().state = TargetState.Point).ignoringDisable(true));
    autoAimTrigger
      .and(allianceZoneTrigger)
      .onTrue(bothShooters(s -> s.getTarget().state = TargetState.Hub).ignoringDisable(true));

    // Pass Point
    autoAimTrigger.and(PBDash.IO_AUTO_PASS::get)
      .whileTrue(bothShooters(s -> s.getTarget().point = FieldUtils.getClosestPassPoint(swerveStateSup.get().Pose.getTranslation())));
    
    // Revving/Idleing as Appropriate
    final Trigger shootActiveTrigger = driver.rightBumper().negate();
    final Trigger hubActiveTrigger = new Trigger(() -> FieldUtils.hubActiveToleranced(FieldUtils.getAlliance(), ControlConstants.preShiftShootMargin, ControlConstants.postShiftShootMargin));
    
    // Idle when right bumper
    driver.rightBumper()
      .whileTrue
      (
        bothShooters(Shooter::idleFlywheels)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      );

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

    // Dual manual shoot
    driver.leftBumper()
      .and(driver.rightBumper().negate())
      .whileTrue
      (
        parallel
        (
          bothShooters(Shooter::makeShootSafe),
          bothShootersCmd(Shooter::runFlywheelsCommand),
          bothShootersCmd(Shooter::runIndexerCommand)
        ).withName("Manual Shoot")
      );

    // Port manual shoot
    driver.rightBumper().negate()
      .and(debug.leftTrigger()
        .or(buttonPad.C1().and(btnSetPass.or(btnSetLocalisation))))
      .and(debug.leftBumper().negate())
      .whileTrue
      (
        parallel
        (
          runOnce(s_PortShooter::makeShootSafe),
          s_PortShooter.runFlywheelsCommand(),
          s_PortShooter.runIndexerCommand()
        ).withName("Manual Shoot Port")
      );

    // Port eject
    buttonPad.A2().and(btnSetPass.or(btnSetLocalisation))
      .whileTrue
      (
        parallel
        (
          s_PortShooter.runIndexerCommand(),
          runOnce(s_PortShooter::idleFlywheels)
        ).withName("Eject Port")
      );

    // Port unjam
    debug.leftBumper()
      .or(buttonPad.C2().and(btnSetPass.or(btnSetLocalisation)))
      .whileTrue
      (
        parallel
        (
          s_PortShooter.runOnce(s_PortShooter::idleFlywheels),
          s_PortShooter.reverseIndexerCommand()
        ).withName("Unjam Port")
      );

    // Stbd manual shoot
    driver.rightBumper().negate()
      .and(debug.rightTrigger()
        .or(buttonPad.F1().and(btnSetPass.or(btnSetLocalisation))))
      .and(debug.rightBumper().negate())
      .whileTrue
      (
        parallel
        (
          runOnce(s_StbdShooter::makeShootSafe),
          s_StbdShooter.runFlywheelsCommand(),
          s_StbdShooter.runIndexerCommand()
        ).withName("Manual Shoot Stbd")
      );

    // Stbd eject
    buttonPad.H2().and(btnSetPass.or(btnSetLocalisation))
      .whileTrue
      (
        parallel
        (
          s_StbdShooter.runIndexerCommand(),
          runOnce(s_StbdShooter::idleFlywheels)
        ).withName("Eject Stbd")
      );

    // Stbd unjam
    debug.rightBumper()
      .or(buttonPad.F2().and(btnSetPass.or(btnSetLocalisation)))
      .whileTrue
      (
        parallel
        (
          s_StbdShooter.runOnce(s_StbdShooter::idleFlywheels),
          s_StbdShooter.reverseIndexerCommand()
        ).withName("Unjam Stbd")
      );

    // Manual Target Control
    // Dual
    autoAimTrigger.negate()
      .whileTrue(s_PortShooter.adjustDistanceCommand(() -> -MathUtil.applyDeadband(debug.getRightY(), ControlConstants.manualShooterDeadband) * ControlConstants.manualShooterDistanceAmount))
      .whileTrue(s_StbdShooter.adjustDistanceCommand(() -> -MathUtil.applyDeadband(debug.getRightY(), ControlConstants.manualShooterDeadband) * ControlConstants.manualShooterDistanceAmount))
      .whileTrue(s_PortShooter.adjustAzimuthCommand(() -> -MathUtil.applyDeadband(debug.getRightX(), ControlConstants.manualShooterDeadband) * ControlConstants.manualShooterAzimuthAmount))
      .whileTrue(s_StbdShooter.adjustAzimuthCommand(() -> -MathUtil.applyDeadband(debug.getRightX(), ControlConstants.manualShooterDeadband)* ControlConstants.manualShooterAzimuthAmount));

    // Port
    buttonPad.B2().and(btnSetManual).and(autoAimTrigger.negate())
      .whileTrue(s_PortShooter.adjustAzimuthCommand(() -> ControlConstants.manualShooterAzimuthAmount));
    buttonPad.D2().and(btnSetManual).and(autoAimTrigger.negate())
      .whileTrue(s_PortShooter.adjustAzimuthCommand(() -> -ControlConstants.manualShooterAzimuthAmount));
    buttonPad.C1().and(btnSetManual).and(autoAimTrigger.negate())
      .whileTrue(s_PortShooter.adjustDistanceCommand(() -> ControlConstants.manualShooterDistanceAmount));
    buttonPad.C2().and(btnSetManual).and(autoAimTrigger.negate())
      .whileTrue(s_PortShooter.adjustDistanceCommand(() -> -ControlConstants.manualShooterDistanceAmount));

    // Stbd
    buttonPad.E2().and(btnSetManual).and(autoAimTrigger.negate())
      .whileTrue(s_StbdShooter.adjustAzimuthCommand(() -> ControlConstants.manualShooterAzimuthAmount));
    buttonPad.G2().and(btnSetManual).and(autoAimTrigger.negate())
      .whileTrue(s_StbdShooter.adjustAzimuthCommand(() -> -ControlConstants.manualShooterAzimuthAmount));
    buttonPad.F1().and(btnSetManual).and(autoAimTrigger.negate())
      .whileTrue(s_StbdShooter.adjustDistanceCommand(() -> ControlConstants.manualShooterDistanceAmount));
    buttonPad.F2().and(btnSetManual).and(autoAimTrigger.negate())
      .whileTrue(s_StbdShooter.adjustDistanceCommand(() -> -ControlConstants.manualShooterDistanceAmount));

    // Manual Flywheel control
    // Rev
    buttonPad.A1().and(btnSetManual)
      .and(driver.rightBumper().negate())
      .onTrue(runOnce(() -> s_PortShooter.makeShootSafe()))
      .whileTrue(s_PortShooter.runFlywheelsCommand());
    buttonPad.H1().and(btnSetManual)
      .and(driver.rightBumper().negate())
      .onTrue(runOnce(() -> s_StbdShooter.makeShootSafe()))
      .whileTrue(s_PortShooter.runFlywheelsCommand());

    // Idle
    buttonPad.A2().and(btnSetManual)
      .or(buttonPad.B2().and(btnSetPass.or(btnSetLocalisation)))
      .onTrue(runOnce(() -> s_PortShooter.idleFlywheels()));
    buttonPad.H2().and(btnSetManual)
      .or(buttonPad.G2().and(btnSetPass.or(btnSetLocalisation)))
      .onTrue(runOnce(() -> s_StbdShooter.idleFlywheels()));

    // Stop
    buttonPad.A3().and(btnSetManual)
      .or(PBDash.E_STOP.asPulse().and(PBDash.E_STOP::get))
      .onTrue(s_PortShooter.setFlySpeedCommand(0))
      .onTrue(runOnce(() -> s_PortShooter.revFlywheels()));
    buttonPad.H3().and(btnSetManual)
      .or(PBDash.E_STOP.asPulse().and(PBDash.E_STOP::get))
      .onTrue(s_StbdShooter.setFlySpeedCommand(0))
      .onTrue(runOnce(() -> s_StbdShooter.revFlywheels()));

    // Reverse
    buttonPad.A4().and(btnSetManual)
      .onTrue(s_PortShooter.setFlySpeedCommand(-FlywheelConstants.idleSpeed))
      .onTrue(runOnce(() -> s_PortShooter.revFlywheels()));
    buttonPad.H4().and(btnSetManual)
      .onTrue(s_StbdShooter.setFlySpeedCommand(-FlywheelConstants.idleSpeed))
      .onTrue(runOnce(() -> s_StbdShooter.revFlywheels()));

    // Manual Feeder control
    // Run
    buttonPad.B4().and(btnSetManual)
      .whileTrue(s_PortShooter.runIndexerCommand());
    buttonPad.G4().and(btnSetManual)
      .whileTrue(s_StbdShooter.runIndexerCommand());
    
    // Stop
    buttonPad.C4().and(btnSetManual)
      .or(buttonPad.B2().and(btnSetPass.or(btnSetLocalisation)))
      .whileTrue(s_PortShooter.stopIndexerCommand());
    buttonPad.F4().and(btnSetManual)
      .or(buttonPad.G2().and(btnSetPass.or(btnSetLocalisation)))
      .whileTrue(s_StbdShooter.stopIndexerCommand());

    // Reverse
    buttonPad.D4().and(btnSetManual)
      .whileTrue(s_PortShooter.reverseIndexerCommand());
    buttonPad.E4().and(btnSetManual)
      .whileTrue(s_StbdShooter.reverseIndexerCommand());


    // -------------INTAKE-------------- //
    // Deploy
    driver.leftTrigger()
      .or(buttonPad.D3().and(btnSetPass.or(btnSetLocalisation)))
      .or(buttonPad.B7().and(btnSetManual))
      .onTrue(s_Intake.extendCommand());

    // Run
    debug.b().negate()
      .and
      (
        debug.a()
          .or(buttonPad.D1().and(btnSetPass.or(btnSetLocalisation)))
          .or(driver.leftTrigger().and(s_Intake::extended).and(PBDash.E_STOP.asTrigger().negate()))
      )
      .whileTrue(s_Intake.runIntakeCommand())
      .onFalse(s_Intake.stopIntakeCommand());

    // Manual run
    buttonPad.A6().and(btnSetManual)
      .whileTrue(s_Intake.runIntakeCommand(PBDash.IO_INTAKE_SPEED::get));

    // Agitate
    driver.povDown()
      .or(debug.x())
      .or(buttonPad.D2().and(btnSetPass.or(btnSetLocalisation)))
      .or(buttonPad.B8().and(btnSetManual))
      .whileTrue(s_Intake.extensionJostleCommand())
      .onFalse(s_Intake.extendCommand());

    // Stow
    driver.povUp()
      .or(buttonPad.E3().and(btnSetPass.or(btnSetLocalisation)))
      .or(buttonPad.C7().and(btnSetManual))
      .onTrue(s_Intake.retractCommand());

    // Reverse
    debug.b()
      .or(buttonPad.E1().and(btnSetPass.or(btnSetLocalisation)))
      .or(buttonPad.A7().and(btnSetManual))
      .onTrue(s_Intake.reverseIntakeCommand())
      .onFalse(s_Intake.stopIntakeCommand());

    // Manual extension
    debug.povDown()
      .or(buttonPad.B6().and(btnSetManual))
      .whileTrue(s_Intake.manualExtensionCommand(() -> ControlConstants.manualIntakeExtensionAmount));
    debug.povUp()
      .or(buttonPad.C6().and(btnSetManual))
      .whileTrue(s_Intake.manualExtensionCommand(() -> -ControlConstants.manualIntakeExtensionAmount));

    // Squish
    buttonPad.E2().and(btnSetPass.or(btnSetLocalisation))
      .or(buttonPad.C8().and(btnSetManual))
      .onTrue(s_Intake.squishCommand())
      .onFalse(s_Intake.extendCommand());


    // -------------CLIMBER------------- //
    // Stow
    debug.back()
      .or(buttonPad.G7().and(btnSetManual))
      .or(switchboard.button(IDConstants.climbButtonID))
      .onTrue(s_Climber.retractCommand());
    // Deploy
    debug.start()
      .or(buttonPad.F7().and(btnSetManual))
      .onTrue(s_Climber.deployCommand());
      
    // Manual extension
    s_Climber.setDefaultCommand(s_Climber.adjustTargetCommand(() -> debug.getLeftY() * ControlConstants.manualClimberExtensionScale));

    buttonPad.F6().and(btnSetManual)
      .whileTrue(s_Climber.adjustTargetCommand(() -> ControlConstants.manualClimberExtensionAmount));
    buttonPad.G6().and(btnSetManual)
      .whileTrue(s_Climber.adjustTargetCommand(() -> -ControlConstants.manualClimberExtensionAmount));


    buttonPad.B3().and(btnSetPass)
      .or(buttonPad.F8().and(btnSetManual))
      .onTrue(runOnce(() -> state.climbPos = ClimbPosition.MidLeft).ignoringDisable(true));
    buttonPad.G3().and(btnSetPass)
      .or(buttonPad.G8().and(btnSetManual))
      .onTrue(runOnce(() -> state.climbPos = ClimbPosition.MidRight).ignoringDisable(true));


    // -------------BTN-PAD------------- //

    buttonPad.setDisplayGrid(ButtonPadConstants.passPointMap);
    buttonPad.setColour(PadColour.OFF, 68, 69, 70);
    buttonPad.setColour(PadColour.FULL_RED, 71);

    //   A B C D E F G H  M
    // 1 [][][][][][][][] ()
    // 2 [][][][][][][][] ()
    // 3 [][][][][][][][] ()
    // 4 [][][][][][][][] ()
    // 5 [][][][][][][][] ()
    // 6 [][][][][][][][] ()
    // 7 [][][][][][][][] ()
    // 8 [][][][][][][][] ()

    // A4-H8 -> Alliance Zone map
    // M1 -> Full auto targeting, reset target -> map sets pass point
    // M2 -> Position Mode -> map sets robot position

    buttonPad.M1().onTrue(runOnce(() -> state.btnSet = ButtonPadState.PassPointSelection).ignoringDisable(true));
    buttonPad.M2()
      .onTrue(runOnce(() -> state.btnSet = ButtonPadState.LocalisationOveride).ignoringDisable(true))
      .onFalse(runOnce(() -> state.btnSet = ButtonPadState.PassPointSelection).ignoringDisable(true));

    buttonPad.M4().onTrue(runOnce(() -> state.btnSet = ButtonPadState.ManualControls).ignoringDisable(true));

    buttonPad.M6().and(PBDash.E_STOP.asTrigger()).onTrue(runOnce(() -> PBDash.E_STOP.put(false)).ignoringDisable(true));
    buttonPad.M8().onTrue(runOnce(() -> PBDash.E_STOP.put(true)).ignoringDisable(true));

    PBDash.E_STOP.asTrigger()
      .onTrue(runOnce(() -> buttonPad.setColour(PadColour.FULL_ORANGE, 69)).ignoringDisable(true))
      .onFalse(runOnce(() -> buttonPad.setColour(PadColour.OFF, 69)).ignoringDisable(true));

    btnSetPass.onTrue(runOnce(() -> buttonPad.setDisplayGrid(ButtonPadConstants.passPointMap)).ignoringDisable(true));
    btnSetLocalisation.onTrue(runOnce(() -> buttonPad.setDisplayGrid(ButtonPadConstants.localisationMap)).ignoringDisable(true));
    btnSetManual.onTrue(runOnce(() -> buttonPad.setDisplayGrid(ButtonPadConstants.manualControlGrid)).ignoringDisable(true));

    for (int x = 0; x <= 3; x++)
    {
      for (int y = 0; y <= 7; y++)
      {
        double targetX = 3.5 - x;
        double targetY = 7.5 - y;
        btnSetPass.and(buttonPad.getBtn(32 + y + (8 * x)))
          .onTrue(bothShooters(s -> s.getTarget().point = new AllianceTranslation2d(targetX, targetY).get()).ignoringDisable(true));
        btnSetLocalisation.and(buttonPad.getBtn(32 + y + (8 * x)))
          .onTrue(runOnce(() -> s_Vision.setPose(new AlliancePose2d(targetX, targetY, 0).get())).ignoringDisable(true));
      }
    }
  }

  /** Mutually exclusive to {@link Controls#bind bind()} */
  public static void bindSysId
  ( 
    CommandXboxController driver,
    JoystickTransmuter driverStick,
    Brake driverBrake,
    CommandSwerveDrivetrain s_Swerve
  )
  {
    s_Swerve.setDefaultCommand
    (
      new ManualDrive
      (
        s_Swerve, 
        driverStick::stickOutput,
        () -> -driver.getRightX(),
        driver::getRightTriggerAxis
      )
    );

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

  private static Command bothShooters(Consumer<Shooter> action)
  {
    return runOnce(() -> {
      action.accept(s_PortShooter);
      action.accept(s_StbdShooter);
    });
  }

  private static Command bothShootersCmd(Function<Shooter, Command> cmd)
  {
    return parallel
    (
      cmd.apply(s_PortShooter),
      cmd.apply(s_StbdShooter)
    );
  }
}
