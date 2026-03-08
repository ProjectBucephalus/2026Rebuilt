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
  private static enum ClimbPosition
  {
    OutLeft, OutRight,
    MidLeft, MidRight,
    InLeft, InRight
  }

  private ButtonPadState btnSet = ButtonPadState.PassPointSelection;
  private ClimbPosition climbPos = ClimbPosition.OutLeft;

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
  private final CommandGenericHID switchboard = new CommandGenericHID(2);
  private final Launchpad buttonPad = new Launchpad(3);
  
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
  }

  /** Sets primary control bindings */
  private void bindControls()
  {
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

    driver.start()
      .onTrue(runOnce(() -> s_Swerve.resetRotation(Rotation2d.kZero)));

    PBDash.IO_FENCE.asSwitch()
      .and(() -> nudging && s_Vision.hasLocalisation())
      .and
      (
            bumpNB.asTrigger()
        .or(bumpSB.asTrigger())
        .or(bumpNR.asTrigger())
        .or(bumpSR.asTrigger())
      )
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
    
    // TODO: if (nudging && in trench zone) {nudge to nearest 180 degrees}

    //driver.b -> ?? bump rotation lock ??
    driver.b()
      .toggleOnTrue(
        new HeadingLockedDrive
        (
          s_Swerve, 
          driverStick::stickOutput,
          Rotation2d.kCCW_90deg,
          Rotation2d.kZero,
          () -> swerveState.Pose
        )
      );
    driver.x()
      .toggleOnTrue(
        new HeadingLockedDrive
        (
          s_Swerve, 
          driverStick::stickOutput,
          Rotation2d.kCW_90deg,
          Rotation2d.kZero,
          () -> swerveState.Pose
        )
      );
    //driver.y -> trench rotation lock -> rotate on press, heading straight towards other zone
    //driver.x -> tower rotation lock -> based on selected clime location, enable attractor
    //driver.a -> outpost rotation lock -> face in or right, whichever is closer on press


    // -------------STATE--------------- //

    final Trigger autoAimTrigger = new Trigger(() -> autoAim && s_Vision.hasLocalisation());

    PBDash.IO_AUTO_AIM.asSwitch()
      .onChange(runOnce(() -> autoAim = PBDash.IO_AUTO_AIM.get()));
    switchboard.button(1/*autoAimSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_AUTO_AIM.put(switchboard.button(0/*autoAimSwitchID*/).getAsBoolean())));
    debug.rightStick().onTrue(runOnce(() -> PBDash.IO_AUTO_AIM.put(false)));

    PBDash.IO_AUTO_PASS.asSwitch()
      .onChange(runOnce(() -> autoPass = PBDash.IO_AUTO_PASS.get()));
    switchboard.button(1/*autoPassSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_AUTO_PASS.put(switchboard.button(0/*autoPassSwitchID*/).getAsBoolean())));
    
    PBDash.IO_AUTO_REV.asSwitch()
      .onChange(runOnce(() -> autoRev = PBDash.IO_AUTO_REV.get()));
    switchboard.button(1/*autoRevSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_AUTO_REV.put(switchboard.button(0/*autoRevSwitchID*/).getAsBoolean())));

    switchboard.button(1/*fencingSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_FENCE.put(switchboard.button(0/*fencingSwitchID*/).getAsBoolean())));
    switchboard.button(1/*visionSwitchID*/)
      .onChange(runOnce(() -> PBDash.IO_LL.put(switchboard.button(0/*visionSwitchID*/).getAsBoolean())));

    driver.back().onTrue(runOnce(() -> nudging = false));
    driver.y().or(driver.b()).onTrue(runOnce(() -> nudging = true));
    driver.back().or(driver.y()).or(driver.b()).onFalse(runOnce(() -> PBDash.STATE_NUDGING.put(nudging)));


    // -------------SHOOTERS------------ //

    /* Targetting States */
    autoAimTrigger
      .onFalse(modifyTargetsCommand(target -> target.state = TargetState.Manual));
    autoAimTrigger
      .and(() -> !FieldUtils.inAllianceZone(getTranslation()))
      .onTrue(modifyTargetsCommand(target -> target.state = TargetState.Point));
    autoAimTrigger
      .and(() -> FieldUtils.inAllianceZone(getTranslation()))
      .onTrue(modifyTargetsCommand(target -> target.state = TargetState.Hub));

    /* Pass Point */
    autoAimTrigger.and(() -> autoPass)
      .whileTrue(modifyTargetsCommand(target -> target.point = FieldUtils.getClosestPassPoint(getTranslation())));
    
    /* Revving/Idleing as Appropriate */
    final Trigger shootActiveTrigger = driver.rightBumper().negate()
      .and(() -> FieldUtils.hubActiveToleranced(FieldUtils.getAlliance(), ControlConstants.preShiftShootMargin, ControlConstants.postShiftShootMargin));
    
    // Idle when right bumper
    driver.rightBumper()
      .whileTrue
      (
        forBothShootersCommand(Shooter::idleFlywheels)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      );

    // Rev if auto aiming, auto revving, and shooters are active
    autoAimTrigger
      .and(() -> autoRev)
      .and(shootActiveTrigger)
      .onTrue(forBothShootersCommand(Shooter::revFlywheels))
      .onFalse(forBothShootersCommand(Shooter::idleFlywheels));

    /* Shooting when Ready */
    shootActiveTrigger
      .and(s_PortShooter::shootReady)
      .and(s_StbdShooter::shootReady)
      .whileTrue(s_Indexer.runCommand(() -> Math.max(Math.max(s_StbdShooter.getSpeed(), s_PortShooter.getSpeed()), FeederConstants.feederMinSpeed)));

    //driver.leftBumper -> manual shoot -> ensure flywheels at least idle speed, then run indexers
    driver.leftBumper()
      .and(driver.rightBumper().negate())
      .whileTrue
      (
        s_Indexer.runCommand(() -> Math.max(Math.max(s_StbdShooter.getSpeed(), s_PortShooter.getSpeed()), FeederConstants.feederMinSpeed))
          //.onlyIf(() -> s_StbdShooter.makeShootSafe() && s_PortShooter.makeShootSafe())
      );

    // debug.leftTrigger -> run port flywheel and indexer, return to previous state on release // ?? what speed ??
    driver.rightBumper().negate()
      .and(debug.leftTrigger()
        .or(buttonPad.G1()))
      .and(debug.leftBumper().negate())
      .whileTrue(s_Indexer.runCommand(s_PortShooter::getSpeed).alongWith(run(s_PortShooter::revFlywheels)));//runOnce(() -> s_PortShooter.revFlywheels()));
    // debug.leftBumper -> port shooter idle, reverse indexer, return to previous state on release
    buttonPad.G2()
        .whileTrue(s_Indexer.runCommand(s_PortShooter::getSpeed).alongWith(run(s_PortShooter::idleFlywheels)));
    debug.leftBumper()
      .or(buttonPad.G3())
      .whileTrue
      (
        parallel
        (
          s_PortShooter.run(s_PortShooter::idleFlywheels),
          s_Indexer.runCommand(() -> FeederConstants.feederReverseSpeed)
        )
      );

    // debug.rightTrigger -> run stbd flywheel and indexer, return to previous state on release // ?? what speed ??
    driver.rightBumper().negate()
      .and(debug.rightTrigger()
        .or(buttonPad.H1()))
      .and(debug.rightBumper().negate())
      .whileTrue(s_Indexer.runCommand(s_StbdShooter::getSpeed).alongWith(run(s_StbdShooter::revFlywheels)));//run(() -> s_StbdShooter.revFlywheels()));
    // debug.rightBumper -> stbd shooter idle, reverse indexer, return to previous state on release 
    buttonPad.H2()
        .whileTrue(s_Indexer.runCommand(s_StbdShooter::getSpeed).alongWith(run(s_StbdShooter::idleFlywheels)));
    debug.rightBumper()
      .or(buttonPad.H3())
      .whileTrue
      (
        parallel
        (
          s_StbdShooter.run(s_StbdShooter::idleFlywheels),
          s_Indexer.runCommand(() -> FeederConstants.feederReverseSpeed)
        )
      );

    /* Manual Control */
    autoAimTrigger.negate()
      .whileTrue(s_PortShooter.adjustDistanceCommand(() -> MathUtil.applyDeadband(debug.getRightY(), ControlConstants.manualShooterDeadband)))
      .whileTrue(s_StbdShooter.adjustDistanceCommand(() -> MathUtil.applyDeadband(debug.getRightY(), ControlConstants.manualShooterDeadband)))
      .whileTrue(s_PortShooter.adjustAzimuthCommand(() -> -MathUtil.applyDeadband(debug.getRightX(), ControlConstants.manualShooterDeadband)))
      .whileTrue(s_StbdShooter.adjustAzimuthCommand(() -> -MathUtil.applyDeadband(debug.getRightX(), ControlConstants.manualShooterDeadband)));


    // -------------INTAKE-------------- //

    driver.leftTrigger()
      .onTrue(s_Hopper.extendCommand());

    debug.b().negate()
      .and
      (
        debug.a()
          .or(driver.leftTrigger().and(s_Hopper::extended))
          .or(buttonPad.B1())
      )
      .whileTrue(s_Hopper.runIntakeCommand())
      .onFalse(s_Hopper.stopIntakeCommand());

    driver.povUp()
      .or(debug.x())
      .or(buttonPad.A2())
      .whileTrue(parallel(s_Hopper.extensionJostleCommand(), s_Hopper.runIntakeCommand()))
      .onFalse(s_Hopper.extendCommand());

    driver.povDown().onTrue(s_Hopper.retractCommand());

    debug.b()
      .or(buttonPad.B3())
      .onTrue(s_Hopper.reverseIntakeCommand())
      .onFalse(s_Hopper.stopIntakeCommand());

    debug.povUp()
      .or(buttonPad.A1())
      .whileTrue(s_Hopper.manualExtensionCommand(() -> ControlConstants.manualIntakeExtensionAmount));
    debug.povDown()
      .or(buttonPad.A3())
      .whileTrue(s_Hopper.manualExtensionCommand(() -> -ControlConstants.manualIntakeExtensionAmount));

    // -------------CLIMBER------------- //
    
    debug.back()
      .or(buttonPad.D2())
      .onTrue(s_Climber.retractCommand());
    debug.start()
      .or(buttonPad.E2())
      .onTrue(s_Climber.deployCommand());
      
    s_Climber.setDefaultCommand(s_Climber.adjustTargetCommand(() -> debug.getLeftY() * ControlConstants.manualClimberExtensionScale));

    switchboard.button(0/*climbButtonID1*/).and(switchboard.button(0/*climbButtonID2*/))
        .onTrue(s_Climber.retractCommand());

    buttonPad.C2().onTrue(runOnce(() -> climbPos = ClimbPosition.MidLeft).ignoringDisable(true));
    buttonPad.D1().onTrue(runOnce(() -> climbPos = ClimbPosition.OutLeft).ignoringDisable(true));
    buttonPad.D3().onTrue(runOnce(() -> climbPos = ClimbPosition.InLeft).ignoringDisable(true));
    buttonPad.E1().onTrue(runOnce(() -> climbPos = ClimbPosition.OutRight).ignoringDisable(true));
    buttonPad.E3().onTrue(runOnce(() -> climbPos = ClimbPosition.InRight).ignoringDisable(true));
    buttonPad.F2().onTrue(runOnce(() -> climbPos = ClimbPosition.MidRight).ignoringDisable(true));


    // -------------BTN-PAD------------- //

    buttonPad.setDisplayGrid(ButtonPadConstants.passPointMap);

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

    Trigger btnSetPass          = new Trigger(() -> btnSet == ButtonPadState.PassPointSelection);
    Trigger btnSetLocalisation  = new Trigger(() -> btnSet == ButtonPadState.LocalisationOveride);
    Trigger btnSetManual        = new Trigger(() -> btnSet == ButtonPadState.ManualControls);
    Trigger btnSetAuto          = new Trigger(() -> btnSet == ButtonPadState.AutoDisplay);

    buttonPad.M1().onTrue(runOnce(() -> btnSet = ButtonPadState.PassPointSelection).ignoringDisable(true));
    buttonPad.M2()
      .onTrue(runOnce(() -> btnSet = ButtonPadState.LocalisationOveride).ignoringDisable(true))
      .onFalse(runOnce(() -> btnSet = ButtonPadState.PassPointSelection).ignoringDisable(true));

    buttonPad.M6().and(PBDash.E_STOP.asSwitch())

    btnSetPass.onTrue(runOnce(() -> buttonPad.setDisplayGrid(ButtonPadConstants.passPointMap)).ignoringDisable(true));
    btnSetLocalisation.onTrue(runOnce(() -> buttonPad.setDisplayGrid(ButtonPadConstants.localisationMap)).ignoringDisable(true));

    for (int x = 0; x <= 3; x++)
    {
      for (int y = 0; y <= 7; y++)
      {
        double targetX = 3.5 - x;
        double targetY = 7.5 - y;
        btnSetPass.and(buttonPad.getBtn(32 + y + (8 * x)))
          .onTrue(modifyTargetsCommand(target -> target.point = new AllianceTranslation2d(targetX, targetY).get()));
        btnSetLocalisation.and(buttonPad.getBtn(32 + y + (8 * x)))
          .onTrue(runOnce(() -> s_Vision.setPose(new AlliancePose2d(targetX, targetY, 0).get())));
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
    return run(() -> {
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
    initInputTransmute();
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
    s_PortShooter.getTarget().speed = PBDash.TEST_FLYSPEED.get();
    //s_PortShooter.getTarget().azimuth = PBDash.TEST_AZIMUTH.get();
    s_PortShooter.getTarget().altitude = PBDash.TEST_ALTITUDE.get();
    s_PortShooter.getTarget().flywheelsActive = true;
    s_StbdShooter.getTarget().speed = PBDash.TEST_FLYSPEED.get();
    //s_StbdShooter.getTarget().azimuth = PBDash.TEST_AZIMUTH.get();
    s_StbdShooter.getTarget().altitude = PBDash.TEST_ALTITUDE.get();
    s_StbdShooter.getTarget().flywheelsActive = true;
  }
}