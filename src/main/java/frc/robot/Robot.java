// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.util.AutoBuilder;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;
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
  private SwerveDriveState swerveState = new SwerveDriveState();
  private Optional<Command> autoCommand = Optional.empty();
  private boolean autoMode = false;

  /* Telemetry and SD */
  private final Telemetry ctreLogger = new Telemetry(SwerveConstants.maxSpeed);
  private final CANBus canBus = new CANBus();

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  
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
    driver.rightBumper().negate()
  );
  
  @Logged(name = "Stbd Shooter")
  private final Shooter s_StbdShooter = new Shooter
  (
    () -> swerveState,
    ShooterConstants.stbdShooterOffset,
    IDConstants.stbdShooterIDs,
    ShooterConstants.TurretConstants.stbdPotOffset,
    false,
    driver.rightBumper().negate()
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
    IDConstants.spindexerCAN,
    IDConstants.intakeCAN, 
    IDConstants.extensionCAN, 
    -1 //IDConstants.extensionLimitDIO
  );
  
  @Logged(name = "Feeder")
  private final VelocityMotor s_Feeder = new VelocityMotor
  (
    IDConstants.feederCAN,
    FeederConstants.feederConfig
  );

  /* Rumble */
  private final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, PBDash.RUMBLE_DRIVER::get);
  private final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, PBDash.RUMBLE_DRIVER::get);
  private final RumbleRequester io_operatorRight  = new RumbleRequester(operator, RumbleType.kRightRumble, PBDash.RUMBLE_OPERATOR::get);
  private final RumbleRequester io_operatorLeft   = new RumbleRequester(operator, RumbleType.kLeftRumble, PBDash.RUMBLE_OPERATOR::get);
  
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
      Math.hypot(swerveState.Speeds.vxMetersPerSecond, swerveState.Speeds.vyMetersPerSecond) >= SwerveConstants.robotSpeedThreshold ? 
      SwerveConstants.robotRadiusCircumscribed : 
      SwerveConstants.robotRadiusInscribed
    );
    FieldObject.setRobotPosSup(this::getTranslation);
    
    driverStick
      .rotated(FieldUtils.isRedAlliance())
      .withFieldObjects(GeoFencing.fieldGeoFence)
      .withBrake(driverBrake)
      .withInputCurve(driverInputCurve)
      .withDeadband(driverDeadband);

    GeoFencing.fieldGeoFence.setActiveCondition(() -> s_Vision.hasLocalisation() && PBDash.FENCE_TOGGLE.get() && PBDash.LL_TOGGLE.get());
  }

  /** Sets primary control bindings */
  private void bindControls()
  {
    /* Default Commands */
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

    new Trigger(s_PortShooter::shootReady)
      .and(s_StbdShooter::shootReady)
      .and(driver.leftTrigger().negate())
      .and(driver.leftBumper().negate())
      .whileTrue
      (
        s_Feeder.runEnd
        (
          () -> s_Feeder.setSpeed(Math.min(s_StbdShooter.getSpeed(), s_PortShooter.getSpeed())),
          () -> s_Feeder.setSpeed(0)
        )
      );

    bumpNB.asTrigger()
      .or(bumpSB.asTrigger())
      .or(bumpNR.asTrigger())
      .or(bumpSR.asTrigger())
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

    operator.povLeft().onTrue
      (runOnce(() -> modifyTargets(target -> target.point = ControlConstants.leftFerryTarget.get())));

    operator.povRight().onTrue
      (runOnce(() -> modifyTargets(target -> target.point = ControlConstants.rightFerryTarget.get())));

    new Trigger(() -> autoMode)
      .and(() -> FieldUtils.inLeftHalf(getTranslation()))
      .onTrue(runOnce(() -> modifyTargets(target -> target.point = ControlConstants.leftFerryTarget.get())))
      .onFalse(runOnce(() -> modifyTargets(target -> target.point = ControlConstants.rightFerryTarget.get())));

    new Trigger(() -> autoMode)
      .and(() -> FieldUtils.inAllianceZone(getTranslation()))
      .onTrue(runOnce(() -> modifyTargets(target -> target.state = TargetState.Hub)))
      .onFalse(runOnce(() -> modifyTargets(target -> target.state = TargetState.Point)));

    // TODO: For testing and bringup

    driver.povLeft()
      .onTrue(
        runOnce(() -> {
          autoMode = false;
          modifyTargets(target -> target.state = TargetState.Manual);
          modifyTargets(target -> target.azimuth = PBDash.getDouble("Test Azimuth"));
          modifyTargets(target -> target.altitude = PBDash.getDouble("Test Altitude"));
        })
      );

    driver.povRight().onTrue(runOnce(() -> autoMode = true));

    driver.povUp().onTrue(s_Hopper.extendCommand());//.whileTrue(s_Hopper.manualExtensionCommand(()->-0.05));
    driver.povDown().onTrue(s_Hopper.retractCommand());//.whileTrue(s_Hopper.manualExtensionCommand(()->0.05));

    driver.x()
      .onTrue(s_Hopper.runSpindexerCommand())
      .onFalse(s_Hopper.stopSpindexerCommand());
    driver.b()
      .onTrue(s_Hopper.reverseSpindexerCommand())
      .onFalse(s_Hopper.stopSpindexerCommand());
    driver.a()
      .onTrue(s_Hopper.runIntakeCommand())
      .onFalse(s_Hopper.stopIntakeCommand());
    driver.y()
      .onTrue(s_Hopper.reverseIntakeCommand())
      .onFalse(s_Hopper.stopIntakeCommand());

    driver.leftTrigger()
      .onTrue(s_Feeder.setSpeedCommand(-30))
      .onFalse(s_Feeder.setSpeedCommand(0));
    driver.leftBumper()
      .whileTrue(s_Feeder.setSpeedCommand(50))
      .onFalse(s_Feeder.setSpeedCommand(0));
  }

  /** Mutually exclusive to bindControls */
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
    updateSwerveState();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit()
  {
    FieldUtils.updateAlliance();
    if (getTranslation().equals(Translation2d.kZero))
    {
      s_Swerve.resetPose(FieldUtils.isRedAlliance() ? FieldConstants.redStartLine : FieldConstants.blueStartLine);
    }
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
    FieldUtils.updateAlliance();

    if (autoCommand.isEmpty())
      compileAuto();

    CommandScheduler.getInstance().schedule(autoCommand.get());
  }

  @Override
  public void teleopInit() 
  {
    autoCommand.ifPresent(Command::cancel);
    autoMode = true;
    modifyTargets(target -> target.state = TargetState.Hub);

    FieldUtils.updateAlliance();
    initInputTransmute();
  }

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();
    autoMode = false;
    modifyTargets(target -> target.state = TargetState.Manual);
  }

  @Override
  public void testPeriodic()
  {
    s_StbdShooter.setManual(PBDash.getDouble("Test Azimuth"), PBDash.getDouble("Test Altitude"));
    s_PortShooter.setManual(PBDash.getDouble("Test Azimuth"), PBDash.getDouble("Test Altitude"));
  }
}