// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.epilogue.Epilogue;
//import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.commands.swerve.*;
import frc.robot.constants.*;
import static frc.robot.constants.Constants.*;
import static frc.robot.constants.IDConstants.*;

import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.Constants.FeederConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.constants.FieldConstants.GeoFencing;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

import java.util.Optional;
import java.util.function.Consumer;

import frc.robot.subsystems.*;
import frc.robot.subsystems.generic.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Target;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AlliancePose2d;
import frc.robot.util.AllianceTranslation2d;
import frc.robot.util.AutoBuilder;
import frc.robot.util.FieldUtils;
import frc.robot.util.Launchpad;
import frc.robot.util.LockableXboxController;
import frc.robot.util.MatchTime;
import frc.robot.util.PBDash;
import frc.robot.util.Launchpad.PadColour;
import frc.robot.util.controlTransmutation.*;
import frc.robot.util.libs.Telemetry;

/**
 * 5985 Robot Super-Structure
 * <p>
 * Coordinate system notes:
 * <ul>
 * <li> Robot Relative:
 *  <ul>
 *  <li> +Fore / -Aft -> X axis in Robot coordinates
 *  <li> +Port / -Stbd -> Y axis in Robot corrdinates
 *  </ul>
 * <li> Field Absolute:
 *  <ul>
 *  <li> +East / -West -> X axis in Field coordinates
 *  <li> +North / -South -> Y axis in Field coordinates
 *  </ul>
 * <li> Driver Relative:
 *  <ul>
 *  <li> In / Out -> From driver perspective, to make their lives easier
 *  <li> Left / Right -> From driver perspective, to make their lives easier
 *  </ul>
 * </ul>
 */
@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot 
{
  /* State */
  private static enum ButtonPadState
  {
    PassPointSelection,
    LocalisationOveride,
    AutoDisplay,
    ManualControls
  }
  public static enum ClimbPosition
  {
    OutLeft, OutRight,
    MidLeft, MidRight,
    InLeft, InRight
  }
  private static enum HeadingLockState { Unlocked, Climb, General }

  private ButtonPadState btnSet = ButtonPadState.PassPointSelection;
  private ClimbPosition climbPos = ClimbPosition.OutLeft;
  private HeadingLockState headingLock = HeadingLockState.Unlocked;

  private SwerveDriveState swerveState = new SwerveDriveState();
  private Optional<Command> autoCommand = Optional.empty();

  private boolean nudging = true;

  private boolean autoAim = false;
  private boolean autoPass = false;
  private boolean autoRev = false;

  /* Telemetry and SD */
  private final Telemetry ctreLogger = new Telemetry(SwerveConstants.maxSpeed);
  private final CANBus canBus = new CANBus();

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final LockableXboxController debug = new LockableXboxController(1, Button.kY);
  private final Launchpad buttonPad = new Launchpad(2);
  private final CommandGenericHID switchboard = new CommandGenericHID(4);
  
  /* Subsystems */
  private final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  
  @Logged(name = "Port Shooter")
  private final Shooter s_PortShooter = new Shooter
  (
    () -> swerveState,
    ShooterConstants.portShooterOffset,
    IDConstants.portShooterIDs,
    ShooterConstants.TurretConstants.portPotOffset,
    true,
    ShooterConstants.HoodConstants.portHomeAngle
  );
  
  @Logged(name = "Stbd Shooter")
  private final Shooter s_StbdShooter = new Shooter
  (
    () -> swerveState,
    ShooterConstants.stbdShooterOffset,
    IDConstants.stbdShooterIDs,
    ShooterConstants.TurretConstants.stbdPotOffset,
    false,
    ShooterConstants.HoodConstants.stbdHomeAngle
  );
  
  @Logged(name = "Vision")
  private final Vision s_Vision = new Vision
  (
    s_Swerve::addVisionMeasurement,
    () -> swerveState.Speeds.omegaRadiansPerSecond,
    new Limelight(portLimelightName, VisionConstants.portLimelightOffset, s_PortShooter::getAzimuth, ShooterConstants.portShooterOffset), 
    new Limelight(stbdLimelightName, VisionConstants.stbdLimelightOffset, s_StbdShooter::getAzimuth, ShooterConstants.stbdShooterOffset)
  );
  
  @Logged(name = "Climber")
  private final LinearExtension s_Climber = new LinearExtension
  (
    IDConstants.climberCAN, 
    IDConstants.climberLimitDIO, 
    0, 
    ClimberConstants.maxPosition, 
    0,
    ClimberConstants.metersPerRotation,
    ClimberConstants.climberConfig
  );
  
  @Logged(name = "Hopper")
  private final Hopper s_Hopper = new Hopper
  (
    IDConstants.intakeCAN, 
    IDConstants.extensionCAN, 
    -1 //IDConstants.extensionLimitDIO
  );

  @Logged(name = "Indexer")
  private final Indexer s_Indexer = new Indexer();

  /* Rumble */
  @SuppressWarnings("unused")
  private final RumbleRequester io_driverRight = new RumbleRequester(driver, RumbleType.kRightRumble, PBDash.RUMBLE_DRIVER::get);
  @SuppressWarnings("unused")
  private final RumbleRequester io_driverLeft  = new RumbleRequester(driver, RumbleType.kLeftRumble, PBDash.RUMBLE_DRIVER::get);
  @SuppressWarnings("unused")
  private final RumbleRequester io_debugRight  = new RumbleRequester(debug, RumbleType.kRightRumble, PBDash.RUMBLE_OPERATOR::get);
  @SuppressWarnings("unused")
  private final RumbleRequester io_debugLeft   = new RumbleRequester(debug, RumbleType.kLeftRumble, PBDash.RUMBLE_OPERATOR::get);
  
  /* Input Transmutation */
  private final JoystickTransmuter driverStick = new JoystickTransmuter(driver::getLeftY, driver::getLeftX).invertX().invertY();
  private final Brake driverBrake = new Brake(driver::getRightTriggerAxis, ControlConstants.maxThrottle, ControlConstants.minThrottle);
  private final InputCurve driverInputCurve = new InputCurve(2);
  private final Deadband driverDeadband = new Deadband();

  public Robot() 
  {
    updateSwerveState();
    
    initLogging();
    initInputTransmute();
    bindControls();
    bindRumbles();
  }

  /* INIT METHODS */
  /* ============ */
  /** Set up logging and telemetry systems */
  private void initLogging() 
  {
    SignalLogger.enableAutoLogging(false);

    if (!isSimulation()) 
    {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }

    Epilogue.bind(this);

    s_Swerve.registerTelemetry(ctreLogger::telemeterize);

    SmartDashboard.putData("Current Commands", CommandScheduler.getInstance());
  }

  /** Set up input modification and fencing systems */
  private void initInputTransmute()
  {
    FieldUtils.activateAllianceFencing();
    FieldObject.setRobotRadiusSup
    (() -> 
      SwerveConstants.robotRadiusExpanded
    );
    FieldObject.setRobotPosSup(this::getTranslation);
    
    driverStick
      .rotated(FieldUtils.isAlliance(Alliance.Red))
      .withFieldObjects(GeoFencing.fieldGeoFence)
      .withBrake(driverBrake)
      .withInputCurve(driverInputCurve)
      .withDeadband(driverDeadband);

    GeoFencing.fieldGeoFence.setActiveCondition(() -> s_Vision.hasLocalisation() && PBDash.IO_FENCE.get());
    GeoFencing.hubBlueOutput
      .setActiveCondition
      (
        () -> 
        FieldUtils.hubActiveToleranced
        (
          Alliance.Blue, 
          ControlConstants.preShiftOutputMargin, 
          ControlConstants.postShiftOutputMargin
        ) && !nudging
      );
    GeoFencing.hubRedOutput
      .setActiveCondition
      (
        () -> 
        FieldUtils.hubActiveToleranced
        (
          Alliance.Red, 
          ControlConstants.preShiftOutputMargin, 
          ControlConstants.postShiftOutputMargin
        ) && !nudging
      );
  }

  /** Sets primary control bindings */
  private void bindControls()
  {
    // -------------STATE--------------- //

    final Trigger autoAimTrigger = new Trigger(() -> autoAim && s_Vision.hasLocalisation());
    final Trigger allianceZoneTrigger = new Trigger(() -> FieldUtils.inAllianceZone(getTranslation()));

    // Button pad modes
    Trigger btnSetPass          = new Trigger(() -> btnSet == ButtonPadState.PassPointSelection);
    Trigger btnSetLocalisation  = new Trigger(() -> btnSet == ButtonPadState.LocalisationOveride);
    Trigger btnSetManual        = new Trigger(() -> btnSet == ButtonPadState.ManualControls);

    // Auto aim switch
    PBDash.IO_AUTO_AIM.asTrigger()
      .onChange(runOnce(() -> autoAim = PBDash.IO_AUTO_AIM.get()).ignoringDisable(true));
    switchboard.button(1/*autoAimSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_AUTO_AIM.put(switchboard.button(0/*autoAimSwitchID*/).getAsBoolean())).ignoringDisable(true));
    debug.rightStick().onTrue(runOnce(() -> PBDash.IO_AUTO_AIM.put(false)).ignoringDisable(true));

    // Auto pass switch
    PBDash.IO_AUTO_PASS.asTrigger()
      .onChange(runOnce(() -> autoPass = PBDash.IO_AUTO_PASS.get()).ignoringDisable(true));
    switchboard.button(1/*autoPassSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_AUTO_PASS.put(switchboard.button(0/*autoPassSwitchID*/).getAsBoolean())).ignoringDisable(true));
    
    // Auto rev switch
    PBDash.IO_AUTO_REV.asTrigger()
      .onChange(runOnce(() -> autoRev = PBDash.IO_AUTO_REV.get()).ignoringDisable(true));
    switchboard.button(1/*autoRevSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_AUTO_REV.put(switchboard.button(0/*autoRevSwitchID*/).getAsBoolean())).ignoringDisable(true));

    // Geofence and Vision switches
    switchboard.button(1/*fencingSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_FENCE.put(switchboard.button(0/*fencingSwitchID*/).getAsBoolean())).ignoringDisable(true));
    switchboard.button(1/*visionSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_LL.put(switchboard.button(0/*visionSwitchID*/).getAsBoolean())).ignoringDisable(true));

    // Nudging
    driver.back().onTrue(runOnce(() -> nudging = false).ignoringDisable(true));
    driver.y().or(driver.b()).onTrue(runOnce(() -> nudging = true).ignoringDisable(true));
    driver.back().or(driver.y()).or(driver.b()).onFalse(runOnce(() -> PBDash.STATE_NUDGING.put(nudging)).ignoringDisable(true));
    
    
    
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

    // Bump nudging
    PBDash.IO_FENCE.asTrigger()
      .and(() -> nudging && s_Vision.hasLocalisation())
      .and
      (
            bumpNB.asTrigger()
        .or(bumpSB.asTrigger())
        .or(bumpNR.asTrigger())
        .or(bumpSR.asTrigger())
      )
      .onTrue(s_Hopper.bumpSafeCommand())
      .whileTrue
      (
        new NonCardinalDrive
        (
          s_Swerve, 
          driverStick::stickOutput, 
          () -> -driver.getRightX(), 
          driver::getRightTriggerAxis, 
          () -> swerveState.Pose.getRotation(), 
          bumpRotationTolerance
        )
      );
    
    // Trench nudging
    PBDash.IO_FENCE.asTrigger()
      .and(() -> nudging && s_Vision.hasLocalisation())
      .and
      (
            trenchNB.asTrigger()
        .or(trenchSB.asTrigger())
        .or(trenchNR.asTrigger())
        .or(trenchSR.asTrigger())
      )
      .whileTrue
      (
        new TrenchNudgeDrive
        (
          s_Swerve, 
          driverStick::stickOutput, 
          () -> -driver.getRightX(), 
          driver::getRightTriggerAxis, 
          () -> swerveState.Pose.getRotation()
        ).onlyIf(() -> headingLock == HeadingLockState.Unlocked)
      );

    // Reset to manual drive when heading unlocks
    new Trigger(() -> headingLock == HeadingLockState.Unlocked)
      .onTrue(s_Swerve.getDefaultCommand());

    // Unlock heading
    driver.axisMagnitudeGreaterThan(XboxController.Axis.kRightX.value, ControlConstants.stickDeadband)
      .onTrue(runOnce(() -> headingLock = HeadingLockState.Unlocked));

    // Lock heading
    driver.y()
      .or(driver.b())
      .or(driver.a())
      .onTrue(runOnce(() -> headingLock = HeadingLockState.General));

    // Bump heading lock
    driver.b().onTrue
    (        
      new NonCardinalDrive
      (
        s_Swerve, 
        driverStick::stickOutput, 
        () -> -driver.getRightX(), 
        driver::getRightTriggerAxis, 
        () -> swerveState.Pose.getRotation(), 
        bumpRotationTolerance
      )
    );

    // Trench heading lock
    driver.y().onTrue(new TrenchLockedDrive(s_Swerve, driverStick::stickOutput, () -> swerveState.Pose));

    // Climb heading lock
    driver.x().onTrue(new ClimbLockedDrive(s_Swerve, driverStick::stickOutput, () -> swerveState.Pose, () -> climbPos));

    // Outpost heading lock
    driver.a().onTrue(new OutpostLockedDrive(s_Swerve, driverStick::stickOutput, () -> swerveState.Pose));

    // Update throttle limits
    PBDash.IO_MAX_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMaxThrottle(PBDash.IO_MAX_THROTTLE.get())));
    PBDash.IO_MIN_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMinThrottle(PBDash.IO_MIN_THROTTLE.get())));


    // -------------SHOOTERS------------ //

    // Targetting States
    autoAimTrigger
      .onFalse(modifyTargetsCommand(target -> target.state = TargetState.Manual).ignoringDisable(true));
    autoAimTrigger
      .and(allianceZoneTrigger.negate())
      .onTrue(modifyTargetsCommand(target -> target.state = TargetState.Point).ignoringDisable(true));
    autoAimTrigger
      .and(allianceZoneTrigger)
      .onTrue(modifyTargetsCommand(target -> target.state = TargetState.Hub).ignoringDisable(true));

    // Pass Point
    autoAimTrigger.and(() -> autoPass)
      .whileTrue(modifyTargetsCommand(target -> target.point = FieldUtils.getClosestPassPoint(getTranslation())));
    
    // Revving/Idleing as Appropriate
    final Trigger shootActiveTrigger = driver.rightBumper().negate();
    final Trigger hubActiveTrigger = new Trigger(() -> FieldUtils.hubActiveToleranced(FieldUtils.getAlliance(), ControlConstants.preShiftShootMargin, ControlConstants.postShiftShootMargin));
    
    // Idle when right bumper
    driver.rightBumper()
      .whileTrue
      (
        forBothShootersCommand(Shooter::idleFlywheels)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      );

    // Rev if auto aiming, auto revving, and shooters are active
    autoAimTrigger
      .and(PBDash.IO_AUTO_REV::get)
      .and(shootActiveTrigger)
      .and(allianceZoneTrigger.and(hubActiveTrigger).or(() -> autoPass))
      .onTrue(forBothShootersCommand(Shooter::revFlywheels))
      .onFalse(forBothShootersCommand(Shooter::idleFlywheels));

    // Shooting when Ready
    shootActiveTrigger
      .and(s_PortShooter::shootReady)
      .and(s_StbdShooter::shootReady)
      .whileTrue(s_Indexer.runCommand(() -> Math.max(Math.max(s_StbdShooter.getSpeed(), s_PortShooter.getSpeed()), FeederConstants.feederMinSpeed)));

    // Dual manual shoot
    driver.leftBumper()
      .and(driver.rightBumper().negate())
      .whileTrue
      (
        s_Indexer.runCommand(() -> Math.max(Math.max(s_StbdShooter.getSpeed(), s_PortShooter.getSpeed()), FeederConstants.feederMinSpeed))
          .alongWith(forBothShootersCommand(Shooter::makeShootSafe))
          .alongWith(forBothShootersCommand(Shooter::revFlywheels))
          .withName("Manual Shoot")
      );

    // Port manual shoot
    driver.rightBumper().negate()
      .and(debug.leftTrigger()
        .or(buttonPad.C1().and(btnSetPass.or(btnSetLocalisation))))
      .and(debug.leftBumper().negate())
      .whileTrue
      (
        s_Indexer.runCommand(s_PortShooter::getSpeed)
        .alongWith(runOnce(s_PortShooter::makeShootSafe))
        .alongWith(runOnce(s_PortShooter::revFlywheels))
        .withName("Manual Shoot Port")
      );

    // Port eject
    buttonPad.A2().and(btnSetPass.or(btnSetLocalisation))
      .whileTrue
      (
        s_Indexer.runCommand(s_PortShooter::getSpeed)
        .alongWith(runOnce(s_PortShooter::idleFlywheels))
        .withName("Eject Port")
      );

    // Port unjam
    debug.leftBumper()
      .or(buttonPad.C2().and(btnSetPass.or(btnSetLocalisation)))
      .whileTrue
      (
        parallel
        (
          s_PortShooter.runOnce(s_PortShooter::idleFlywheels),
          s_Indexer.runCommand(() -> FeederConstants.feederReverseSpeed)
        )
        .withName("Unjam Port")
      );

    // Stbd manual shoot
    driver.rightBumper().negate()
      .and(debug.rightTrigger()
        .or(buttonPad.F1().and(btnSetPass.or(btnSetLocalisation))))
      .and(debug.rightBumper().negate())
      .whileTrue
      (
        s_Indexer.runCommand(s_StbdShooter::getSpeed)
        .alongWith(runOnce(s_StbdShooter::makeShootSafe))
        .alongWith(runOnce(s_StbdShooter::revFlywheels))
        .withName("Manual Shoot Stbd")
      );

    // Stbd eject
    buttonPad.H2().and(btnSetPass.or(btnSetLocalisation))
      .whileTrue
      (
        s_Indexer.runCommand(s_StbdShooter::getSpeed)
        .alongWith(runOnce(s_StbdShooter::idleFlywheels))
        .withName("Eject Stbd")
      );

    // Stbd unjam
    debug.rightBumper()
      .or(buttonPad.F2().and(btnSetPass.or(btnSetLocalisation)))
      .whileTrue
      (
        parallel
        (
          s_StbdShooter.runOnce(s_StbdShooter::idleFlywheels),
          s_Indexer.runCommand(() -> FeederConstants.feederReverseSpeed)
        )
        .withName("Unjam Stbd")
      );

    // Manual Target Control
    // Dual
    autoAimTrigger.negate()
      .whileTrue(s_PortShooter.adjustDistanceCommand(() -> MathUtil.applyDeadband(debug.getRightY(), ControlConstants.manualShooterDeadband)))
      .whileTrue(s_StbdShooter.adjustDistanceCommand(() -> MathUtil.applyDeadband(debug.getRightY(), ControlConstants.manualShooterDeadband)))
      .whileTrue(s_PortShooter.adjustAzimuthCommand(() -> -MathUtil.applyDeadband(debug.getRightX(), ControlConstants.manualShooterDeadband)))
      .whileTrue(s_StbdShooter.adjustAzimuthCommand(() -> -MathUtil.applyDeadband(debug.getRightX(), ControlConstants.manualShooterDeadband)));

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
      .onTrue(runOnce(() -> s_PortShooter.revFlywheels()));
    buttonPad.H1().and(btnSetManual)
      .and(driver.rightBumper().negate())
      .onTrue(runOnce(() -> s_StbdShooter.makeShootSafe()))
      .onTrue(runOnce(() -> s_StbdShooter.revFlywheels()));

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
      .whileTrue(s_Indexer.runCommand(s_PortShooter::getSpeed));
    buttonPad.G4().and(btnSetManual)
      .whileTrue(s_Indexer.runCommand(s_StbdShooter::getSpeed));
    
    // Stop
    buttonPad.C4().and(btnSetManual)
      .or(buttonPad.B2().and(btnSetPass.or(btnSetLocalisation)))
      .whileTrue(s_Indexer.runCommand(() -> 0));
    buttonPad.F4().and(btnSetManual)
      .or(buttonPad.G2().and(btnSetPass.or(btnSetLocalisation)))
      .whileTrue(s_Indexer.runCommand(() -> 0));

    // Reverse
    buttonPad.D4().and(btnSetManual)
      .whileTrue(s_Indexer.runCommand(() -> FeederConstants.feederReverseSpeed));
    buttonPad.E4().and(btnSetManual)
      .whileTrue(s_Indexer.runCommand(() -> FeederConstants.feederReverseSpeed));


    // -------------INTAKE-------------- //
    // Deploy
    driver.leftTrigger()
      .or(buttonPad.D3().and(btnSetPass.or(btnSetLocalisation)))
      .or(buttonPad.B7().and(btnSetManual))
      .onTrue(s_Hopper.extendCommand());

    // Run
    debug.b().negate()
      .and
      (
        debug.a()
          .or(buttonPad.D1().and(btnSetPass.or(btnSetLocalisation)))
          .or(driver.leftTrigger().and(s_Hopper::extended).and(PBDash.E_STOP.asTrigger().negate()))
      )
      .whileTrue(s_Hopper.runIntakeCommand())
      .onFalse(s_Hopper.stopIntakeCommand());

    // Manual run
    buttonPad.A6().and(btnSetManual)
      .whileTrue(s_Hopper.runIntakeCommand(PBDash.IO_INTAKE_SPEED::get));

    // Agitate
    driver.povDown()
      .or(debug.x())
      .or(buttonPad.D2().and(btnSetPass.or(btnSetLocalisation)))
      .or(buttonPad.B8().and(btnSetManual))
      .whileTrue(s_Hopper.extensionJostleCommand())
      .onFalse(s_Hopper.extendCommand());

    // Stow
    driver.povUp()
      .or(buttonPad.E3().and(btnSetPass.or(btnSetLocalisation)))
      .or(buttonPad.C7().and(btnSetManual))
      .onTrue(s_Hopper.retractCommand());

    // Reverse
    debug.b()
      .or(buttonPad.E1().and(btnSetPass.or(btnSetLocalisation)))
      .or(buttonPad.A7().and(btnSetManual))
      .onTrue(s_Hopper.reverseIntakeCommand())
      .onFalse(s_Hopper.stopIntakeCommand());

    // Manual extension
    debug.povDown()
      .or(buttonPad.B6().and(btnSetManual))
      .whileTrue(s_Hopper.manualExtensionCommand(() -> ControlConstants.manualIntakeExtensionAmount));
    debug.povUp()
      .or(buttonPad.C6().and(btnSetManual))
      .whileTrue(s_Hopper.manualExtensionCommand(() -> -ControlConstants.manualIntakeExtensionAmount));

    // Squish
    buttonPad.E2().and(btnSetPass.or(btnSetLocalisation))
      .or(buttonPad.C8().and(btnSetManual))
      .whileTrue(s_Hopper.manualExtensionCommand(() -> ControlConstants.intakeSquishAmount));


    // -------------CLIMBER------------- //
    // Stow
    debug.back()
      .or(buttonPad.G7().and(btnSetManual))
      .or(switchboard.button(0/*climbButtonID1*/).and(switchboard.button(0/*climbButtonID2*/)))
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
      .onTrue(runOnce(() -> climbPos = ClimbPosition.MidLeft).ignoringDisable(true));
    buttonPad.G3().and(btnSetPass)
      .or(buttonPad.G8().and(btnSetManual))
      .onTrue(runOnce(() -> climbPos = ClimbPosition.MidRight).ignoringDisable(true));


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

    buttonPad.M1().onTrue(runOnce(() -> btnSet = ButtonPadState.PassPointSelection).ignoringDisable(true));
    buttonPad.M2()
      .onTrue(runOnce(() -> btnSet = ButtonPadState.LocalisationOveride).ignoringDisable(true))
      .onFalse(runOnce(() -> btnSet = ButtonPadState.PassPointSelection).ignoringDisable(true));

    buttonPad.M4().onTrue(runOnce(() -> btnSet = ButtonPadState.ManualControls).ignoringDisable(true));

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
          .onTrue(modifyTargetsCommand(target -> target.point = new AllianceTranslation2d(targetX, targetY).get()).ignoringDisable(true));
        btnSetLocalisation.and(buttonPad.getBtn(32 + y + (8 * x)))
          .onTrue(runOnce(() -> s_Vision.setPose(new AlliancePose2d(targetX, targetY, 0).get())).ignoringDisable(true));
      }
    }
  }

  /** Mutually exclusive to bindControls */
  @SuppressWarnings("unused")
  private void bindSysIdControls()
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

  /** Sets trigger conditions to activate controller rumbles */
  private void bindRumbles()
  {

  }

  /* UTIL METHODS */
  /* ============ */
  /**
   * Perform some modification on the targets of both shooters
   * 
   * @param updater The action to perform on the targets
   */
  private void modifyTargets(Consumer<Target> updater)
  {
    updater.accept(s_PortShooter.getTarget());
    updater.accept(s_StbdShooter.getTarget());
  }

  private Command forBothShootersCommand(Consumer<Shooter> action)
  {
    return runOnce(() -> {
      action.accept(s_PortShooter);
      action.accept(s_StbdShooter);
    });
  }

  /**
   * Perform some modification on the targets of both shooters
   * 
   * @param updater The action to perform on the targets
   */
  private Command modifyTargetsCommand(Consumer<Target> updater)
  {
    return runOnce(() -> modifyTargets(updater));
  }

  /** Pull current state from drivebase for external use, to avoid repeated expensive calls */
  private void updateSwerveState()
  {
    swerveState = s_Swerve.getState();
    PBDash.FIELD.setRobotPose(swerveState.Pose);
    PBDash.putDouble("Robot Forward Speed metres per second", swerveState.Speeds.vxMetersPerSecond);
  }
  
  private void handleAutoErr(String invalidInstr)
    {PBDash.AUTO_ERRS.put(PBDash.AUTO_ERRS.get() + ", " + invalidInstr);}

  private void compileAuto()
    {autoCommand = Optional.of(AutoBuilder.compileAutoString(PBDash.AUTO_STRING.get(), s_Swerve, () -> swerveState, this::handleAutoErr));}

  /** Returns the t2d of the robot centre in field coordinates */
  public Translation2d getTranslation()
    {return swerveState.Pose.getTranslation();}

  /** Returns the r2d of the robot in field coordinates */
  public Rotation2d getRotation() 
    {return swerveState.Pose.getRotation();}

  @Logged(name = "CAN Load")
  public float getCanLoad() 
    {return canBus.getStatus().BusUtilization;}

  @Logged(name = "Pigeon Degrees")
  public double getPigeonReading() 
    {return swerveState.RawHeading.getDegrees();}
  
  /* OPMODE METHODS */
  /* ============ */
  @Override
  public void robotPeriodic() 
  {
    FieldUtils.updateAutoWinner();
    updateSwerveState();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit()
  {
    FieldUtils.updateAlliance();
    if (getTranslation().equals(Translation2d.kZero))
      s_Swerve.resetPose
      (
        switch (FieldUtils.getAlliance()) 
        {
          case Blue -> FieldConstants.blueStartLine; 
          case Red -> FieldConstants.redStartLine;
        }
      );
  }

  @Override
  public void disabledPeriodic()
  {
    FieldUtils.updateAlliance();

    if (PBDash.AUTO_STRING.hasChanged()) 
      compileAuto();
  }

  @Override
  public void autonomousInit() 
  {
    MatchTime.startAuto();
    FieldUtils.updateAlliance();

    autoAim = true;
    autoRev = true;
    nudging = false;
    if (autoCommand.isEmpty())
      compileAuto();

    CommandScheduler.getInstance().schedule(autoCommand.get());
  }

  @Override
  public void teleopInit() 
  {
    MatchTime.startTele();
    autoCommand.ifPresent(Command::cancel);

    FieldUtils.updateAlliance();
    autoAim = true;
    autoRev = true;
    nudging = true;
    initInputTransmute();

    CommandScheduler.getInstance()
      .schedule
      (
        io_driverLeft.timedRequestCommand("Teleop Start", 1.5), 
        io_driverRight.timedRequestCommand("Teleop Start", 1.5)
      );
  }

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();

    FieldUtils.updateAlliance();
    initInputTransmute();
  }

  @Override
  public void testPeriodic()
  {
    if (!PBDash.E_STOP.get())
    {
      modifyTargets(target -> target.state = TargetState.Manual);
      modifyTargets(target -> target.altitude = PBDash.TEST_ALTITUDE.get());
      modifyTargets(target -> target.speed = PBDash.TEST_FLYSPEED.get());
    }
  }
}