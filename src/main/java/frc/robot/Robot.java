// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.autobuilder.AutoBuilder;
import frc.robot.constants.*;
import frc.robot.constants.Constants.*;
import frc.robot.constants.Constants.IntakeConstants.ExtensionConstants;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.controlTransmutation.*;
import frc.robot.leds.Block;
import frc.robot.leds.patterns.AlternatingPattern;
import frc.robot.leds.patterns.Patterns;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.RollerState;
import frc.robot.subsystems.generic.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.*;

import frc.robot.util.*;

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
  public enum ClimbPosition { None, Left, Right }
  public enum ShootersState { Auto, Stbd, Port, Manual, Test }
  public enum NavState      { Manual, Nudged, Blocked, Following, AtTarget, RedShift, BlueShift, DualShift, Disabled }

  @Logged
  public class RobotState 
  {
    public ClimbPosition climbPos = ClimbPosition.None;
    public ShootersState shoot = ShootersState.Auto;
    public NavState nav = NavState.Disabled;
    public boolean nudging = true;
    public SwerveDriveState swerve = new SwerveDriveState();
  }
  
  @Logged
  public final RobotState state = new RobotState();
  private Optional<Command> autoCommand = Optional.empty();

  /* Telemetry and SD */
  private final CANBus canBus = new CANBus();

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(IDConstants.driverPort);
  private final CommandXboxController operator = new CommandXboxController(IDConstants.debugPort);
  private final CommandGenericHID switchboard = new CommandGenericHID(IDConstants.switchboardPort);
  
  /* Subsystems */
  private final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  
  @Logged(name = "Port Shooter")
  private final Shooter s_PortShooter = new Shooter
  (
    () -> state.swerve,
    ShooterConstants.portShooterOffset,
    IDConstants.portShooterIDs,
    ShooterConstants.TurretConstants.portPotOffset,
    true,
    ShooterConstants.HoodConstants.portHomeAngle,
    true
  );
  
  @Logged(name = "Stbd Shooter")
  private final Shooter s_StbdShooter = new Shooter
  (
    () -> state.swerve,
    ShooterConstants.stbdShooterOffset,
    IDConstants.stbdShooterIDs,
    ShooterConstants.TurretConstants.stbdPotOffset,
    false,
    ShooterConstants.HoodConstants.stbdHomeAngle,
    false
  );

  private final Limelight s_PhotonPort = new Limelight
  (
    IDConstants.portLimelightName, 
    VisionConstants.flatCameraToTurret, 
    s_PortShooter::getAzimuthTimestamped, 
    ShooterConstants.portShooterOffset
  );

  private final Limelight s_PhotonStbd = new Limelight
  (
    IDConstants.stbdLimelightName, 
    VisionConstants.flatCameraToTurret, 
    s_StbdShooter::getAzimuthTimestamped, 
    ShooterConstants.stbdShooterOffset
  );
  
  @Logged(name = "Vision")
  private final Vision s_Vision = new Vision
  (
    s_Swerve::addVisionMeasurement,
    () -> state.swerve.Speeds.omegaRadiansPerSecond,
    s_PhotonPort,
    s_PhotonStbd
  );
  
  @Logged(name = "Climber")
  private final LinearExtension s_Climber = new LinearExtension
  (
    IDConstants.climberCAN, 
    IDConstants.climberLimitDIO, 
    true,
    ClimberConstants.minPosition, 
    ClimberConstants.maxPosition, 
    ClimberConstants.homePosition, 
    ClimberConstants.metersPerRotation,
    ClimberConstants.climberConfig
  );
  
  @Logged(name = "Intake")
  private final Intake s_Intake = new Intake();

  @Logged(name = "Extension")
  private final PositionMotor s_Extension = new PositionMotor(IDConstants.extensionCAN, ExtensionConstants.extensionConfig);

  @Logged(name = "Climb Post Sensor")
  private final DigitalInput io_ClimberPost = new DigitalInput(IDConstants.climberPostDIO);

  @Logged(name = "Extension Encoder")
  private final CANcoder io_ExtensionEncoder = new CANcoder(IntakeConstants.ExtensionConstants.extensionConfig.Feedback.FeedbackRemoteSensorID);

  /* Rumble */
  private final RumbleRequester io_driverRight = new RumbleRequester(driver, RumbleType.kRightRumble, PBDash.RUMBLE_DRIVER::get);
  private final RumbleRequester io_driverLeft  = new RumbleRequester(driver, RumbleType.kLeftRumble, PBDash.RUMBLE_DRIVER::get);
  private final RumbleRequester io_operatorRight  = new RumbleRequester(operator, RumbleType.kRightRumble, PBDash.RUMBLE_OPERATOR::get);
  private final RumbleRequester io_operatorLeft   = new RumbleRequester(operator, RumbleType.kLeftRumble, PBDash.RUMBLE_OPERATOR::get);
  
  /* LEDs */
  private LEDStrip io_LEDs;

  /* Input Transmutation */
  private final JoystickTransmuter driverStick = new JoystickTransmuter(driver::getLeftY, driver::getLeftX).invertX().invertY();
  private final JoystickTransmuter driverStickRaw = new JoystickTransmuter(driver::getLeftY, driver::getLeftX).invertX().invertY();
  private final Brake driverBrake = new Brake
  (
    () -> Math.max
      (
        driver.leftBumper().getAsBoolean() 
          ? ControlConstants.brakeFromIntake 
          : 0, 
        driver.getRightTriggerAxis()
      ), 
    ControlConstants.maxThrottle, 
    ControlConstants.minThrottle
  );
  private final InputCurve driverInputCurve = new InputCurve(2);
  private final Deadband driverDeadband = new Deadband();

  private final AutoBuilder autoBuilder = new AutoBuilder(s_Intake, s_Extension, s_Climber, io_ClimberPost, state);

  public Robot() 
  {    
    initLogging();
    initInputTransmute();

    new ControlBinder
    (
      state, 
      driver, 
      operator, 
      switchboard, 
      driverStick, 
      driverBrake, 
      s_Swerve, 
      s_Vision,
      s_PhotonPort, 
      s_PhotonStbd,
      s_PortShooter, 
      s_StbdShooter, 
      s_Intake, 
      s_Extension,
      s_Climber,
      io_ClimberPost
    )
    .bind();

    bindRumbles();
    bindLEDs();
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

    s_Swerve.registerTelemetry(this::updateSwerveState);

    CameraServer.startAutomaticCapture();

    PBDash.putSendable("Current Commands", CommandScheduler.getInstance());
  }

  /** Set up input modification and fencing systems */
  private void initInputTransmute()
  {
    DriveBuilder.init
    (
      s_Swerve, 
      driverStick::stickOutput,
      () -> -driver.getRightX(),
      driver::getRightTriggerAxis,
      () -> state.swerve.Pose
    );

    driverStick
      .rotated(FieldUtils.isAlliance(Alliance.Red))
      .withFieldObjects(GeoFencing.fieldGeoFence)
      .withBrake(driverBrake)
      .withInputCurve(driverInputCurve)
      .withDeadband(driverDeadband);

    FieldObject.setRobotRadiusSup(() -> SwerveConstants.robotRadiusExpanded);
    FieldObject.setRobotPosSup(() -> state.swerve.Pose.getTranslation());

    FieldObject.setGlobalActiveCondition(() -> s_Vision.hasLocalisation() && PBDash.IO_FENCE.get());
    
    GeoFencing.fieldRedGeoFence.setActiveCondition(() -> FieldUtils.isAlliance(Alliance.Red));
    GeoFencing.fieldBlueGeoFence.setActiveCondition(() -> FieldUtils.isAlliance(Alliance.Blue));
    
    GeoFencing.towerClearBlueLeft .setActiveCondition(() -> state.climbPos == ClimbPosition.Right);
    GeoFencing.towerPostBlueS     .setActiveCondition(() -> state.climbPos != ClimbPosition.Right);
    GeoFencing.towerClearRedLeft  .setActiveCondition(() -> state.climbPos == ClimbPosition.Right);
    GeoFencing.towerPostRedN      .setActiveCondition(() -> state.climbPos != ClimbPosition.Right);
    GeoFencing.towerClearBlueRight.setActiveCondition(() -> state.climbPos == ClimbPosition.Left);
    GeoFencing.towerPostBlueN     .setActiveCondition(() -> state.climbPos != ClimbPosition.Left);
    GeoFencing.towerClearRedRight .setActiveCondition(() -> state.climbPos == ClimbPosition.Left);
    GeoFencing.towerPostRedS      .setActiveCondition(() -> state.climbPos != ClimbPosition.Left);
    GeoFencing.climbBarrier       .setActiveCondition(() -> state.climbPos != ClimbPosition.None);
    
    // Climb attractor TriggerVector setup
    GeoFencing.climbBlueLeft 
      .withControlInput(driverStickRaw::stickOutput)
      .setActiveCondition(() -> state.climbPos == ClimbPosition.Left  && FieldUtils.isAlliance(Alliance.Blue));
    GeoFencing.climbRedLeft  
      .withControlInput(driverStickRaw::stickOutput)
      .setActiveCondition(() -> state.climbPos == ClimbPosition.Left  && FieldUtils.isAlliance(Alliance.Red));
    GeoFencing.climbBlueRight
      .withControlInput(driverStickRaw::stickOutput)
      .setActiveCondition(() -> state.climbPos == ClimbPosition.Right && FieldUtils.isAlliance(Alliance.Blue));
    GeoFencing.climbRedRight 
      .withControlInput(driverStickRaw::stickOutput)
      .setActiveCondition(() -> state.climbPos == ClimbPosition.Right && FieldUtils.isAlliance(Alliance.Red));
  }

  /** Sets trigger conditions to activate controller rumbles */
  private void bindRumbles()
  {
    // See teleopInit/testInit for rumble on teleop start

    new Trigger(() -> FieldUtils.hubActiveToleranced(3, 0)) 
      .onChange(io_driverLeft.timedRumbleCmd("Shift Warning", 3));

    new Trigger(FieldUtils::hubActive) 
      .onChange(io_driverRight.timedRumbleCmd("Shift Change", 1.5));

    new Trigger(() -> MatchTime.getGameTimeRemaining() <= 30)
      .onTrue(io_operatorLeft.timedRumbleCmd("Endgame Start", 3));

    new Trigger(() -> MatchTime.getGameTimeRemaining() <= ControlConstants.lastClimbChance)
      .onTrue(io_operatorRight.timedRumbleCmd("Last Climb Chance", 1.5));
  }

  private void bindLEDs()
  {
    // Port shooter state
    Block.setPatternMulti
    (
      Patterns.supplied
      (() -> 
        switch (s_PortShooter.shootStatus()) 
        {
          case Idling -> Color.kMagenta;
          case BadLocation -> Color.kRed;
          case Aiming -> Color.kYellow;
          case Revving -> Color.kWhite;
          case AwaitingInput -> Color.kBlue;
          case Fire -> Color.kGreen;
          case Vision -> Color.kLimeGreen;
        }
      ),
      IDConstants.portLEDBlocks
    );

    // Stbd shooter state
    Block.setPatternMulti
    (
      Patterns.supplied
      (() -> 
        switch (s_StbdShooter.shootStatus()) 
        {
          case Idling -> Color.kMagenta;
          case BadLocation -> Color.kRed;
          case Aiming -> Color.kYellow;
          case Revving -> Color.kWhite;
          case AwaitingInput -> Color.kBlue;
          case Fire -> Color.kGreen;
          case Vision -> Color.kLimeGreen;
        }
      ),
      IDConstants.stbdLEDBlocks
    );
    
    // Drivebase state
    Block.setPatternMulti
    (
      Patterns.conditional
      (
        () -> FieldUtils.hubBothTransition(3), // both hubs will be active
        new AlternatingPattern(Color.kWhite, Color.kBlack, 2.0),
        Patterns.conditional
        (
          () -> FieldUtils.hubTransition(Alliance.Red, 3), // red hub will be active
          new AlternatingPattern(Color.kRed, Color.kBlack, 2.0),
          Patterns.conditional
          (
            () -> FieldUtils.hubTransition(Alliance.Blue, 3), // blue hub will be active
            new AlternatingPattern(Color.kBlue, Color.kBlack, 2.0),
            Patterns.conditional
            (
              () -> state.nav == NavState.Disabled,
              new AlternatingPattern(Color.kLimeGreen, Color.kGold, 0.5),
              Patterns.supplied
              (() -> 
                switch (state.nav) 
                {
                  // Manual, Nudged, Blocked, Following, AtTarget, RedShift, BlueShift, DualShift
                  default -> Color.kMagenta;
                  case Nudged -> Color.kYellow;
                  case Blocked -> Color.kOrange;
                  case Following -> Color.kYellow;
                  case AtTarget -> Color.kGreen;
                  case RedShift -> Color.kRed;
                  case BlueShift -> Color.kBlue;
                  case DualShift -> Color.kWhite;
                }
              )
            )
          )
        )
      ),
      // Patterns.supplied
      // (() -> 
      //   switch (driveStatus()) 
      //   {
      //     case Both Hubs will be Active -> Alternating White
      //     case Red Hub will be Active -> Red Alternating
      //     case Blue Hub will be Active -> Blue Alternating
      //     case Both Hubs active -> White
      //     case Red Hub active -> Red
      //     case Blue Hub active -> BLue
      //     case Robot in nudge zone & effected -> Yellow
      //     case Robot affected by attractor -> Yellow
      //     case Robot at attractor target -> Green
      //     case Robot Blocked by fence -> Orange
      //   }
      // ),
      IDConstants.lowerLEDBlocks
    );

    io_LEDs = new LEDStrip
    (
      IDConstants.LEDPWDPort, 
      IDConstants.allLEDBlocks
    );
  }

  /* UTIL METHODS */
  /* ============ */

  /** Pull current state from drivebase for external use, to avoid repeated expensive calls */
  private void updateSwerveState(SwerveDriveState swerveState)
  {
    state.swerve = swerveState;
    PBDash.FIELD.setRobotPose(swerveState.Pose);
    PBDash.POSE.put
    (
      String.format
      (
        "X: %.2fm, Y: %.2fm, R: %.0f", 
        swerveState.Pose.getX(), 
        swerveState.Pose.getY(), 
        swerveState.Pose.getRotation().getDegrees()
      )
    );
  }

  private void compileAuto()
  {
    autoCommand = Optional.of(autoBuilder.compile(PBDash.AUTO_STRING.get()));
  }

  private void checkDevices()
  {
    PBDash.DEVICE_ERRORS.init();
    
    if (!driver.isConnected() || !operator.isConnected()) PBDash.DEVICE_ERRORS.append("Controller, ");
    if (!(switchboard.button(1)).or(switchboard.button(2)).or(switchboard.button(3)).getAsBoolean()) PBDash.DEVICE_ERRORS.append("Switchboard, ");

    if (!s_Swerve.devicesValid()) PBDash.DEVICE_ERRORS.append("Drivebase, ");
    
    if (!s_PortShooter.devicesValid()) PBDash.DEVICE_ERRORS.append("Port Shooter, ");
    if (!s_PortShooter.potValid()) PBDash.DEVICE_ERRORS.append("Port Pot, ");
    if (!s_StbdShooter.devicesValid()) PBDash.DEVICE_ERRORS.append("Stbd Shooter, ");
    if (!s_StbdShooter.potValid()) PBDash.DEVICE_ERRORS.append("Stbd Pot, ");
    
    if (!s_Climber.devicesValid()) PBDash.DEVICE_ERRORS.append("Climber Motor, ");
    if (!s_Climber.atLimit()) PBDash.DEVICE_ERRORS.append("Climber Limit Sensor, ");
    if (io_ClimberPost.get()) PBDash.DEVICE_ERRORS.append("Climber Post Sensor, ");
    
    if (!s_Intake.devicesValid()) PBDash.DEVICE_ERRORS.append("Intake Roller, ");
    if (!s_Extension.devicesValid()) PBDash.DEVICE_ERRORS.append("Extension, ");
    if (!io_ExtensionEncoder.isConnected()) PBDash.DEVICE_ERRORS.append("Extension Encoder, ");
    
    if (!s_Vision.hasLocalisation()) PBDash.DEVICE_ERRORS.append("Vision, ");

    double batteryVoltage = Math.round(100 * RobotController.getBatteryVoltage()) / 100.0;
    if (batteryVoltage < 12.5) PBDash.DEVICE_ERRORS.append("Battery " + batteryVoltage + "v, ");
  }

  @Logged(name = "CAN Load")
  public float getCanLoad() 
    {return canBus.getStatus().BusUtilization;}
  
  /* OPMODE METHODS */
  /* ============ */
  @Override
  public void robotPeriodic() 
  {
    FieldUtils.updateAutoWinner();
    PBDash.updateSendables();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit()
  {
    FieldUtils.updateAlliance();
    
    if (state.swerve.Pose.getTranslation().equals(Translation2d.kZero))
      s_Swerve.resetPose
      (
        switch (FieldUtils.getAlliance()) 
        {
          case Blue -> FieldConstants.blueStartLine; 
          case Red -> FieldConstants.redStartLine;
        }
      );

    s_Intake.state = RollerState.Off;
  }

  @Override
  public void disabledPeriodic()
  {
    FieldUtils.updateAlliance();

    if (PBDash.AUTO_STRING.hasChanged()) 
      compileAuto();

    checkDevices();
  }

  @Override
  public void autonomousInit() 
  {
    MatchTime.startAuto();
    FieldUtils.updateAlliance();
    
    compileAuto();

    CommandScheduler.getInstance().schedule(autoCommand.get());
  }

  @Override
  public void teleopInit() 
  {
    MatchTime.startTele();
    FieldUtils.updateAlliance();
    // Update driver input rotation based on alliance
    driverStick.rotated(FieldUtils.isAlliance(Alliance.Red));
    
    autoCommand.ifPresent(Command::cancel);

    PBDash.removeFieldObject("Auto Path");

    CommandScheduler.getInstance()
      .schedule
      (
        io_driverLeft.timedRumbleCmd("Teleop Start", 1.5), 
        io_driverRight.timedRumbleCmd("Teleop Start", 1.5)
      );
  }

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();

    FieldUtils.updateAlliance();
    // Update driver input rotation based on alliance
    driverStick.rotated(FieldUtils.isAlliance(Alliance.Red));

    CommandScheduler.getInstance()
      .schedule
      (
        io_driverLeft.timedRumbleCmd("Test Start", 0.5), 
        io_driverRight.timedRumbleCmd("Test Start", 0.5)
      );
  }

  @Override
  public void testPeriodic()
  {
    state.shoot = ShootersState.Test;
  }
}