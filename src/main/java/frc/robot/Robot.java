// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.commands.swerve.*;
import frc.robot.constants.*;
import static frc.robot.constants.Constants.*;
import static frc.robot.constants.IDConstants.*;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.constants.FieldConstants.GeoFencing;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

import java.util.function.Consumer;

import frc.robot.subsystems.*;
import frc.robot.subsystems.generic.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Target;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AutoFactories;
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
@Logged
public class Robot extends TimedRobot 
{
  /* State */
  private SwerveDriveState swerveState = new SwerveDriveState();
  private Command autoCommand;

  /* Telemetry and SD */
  private final Telemetry ctreLogger = new Telemetry(SwerveConstants.maxSpeed);
  private final CANBus canBus = new CANBus();
  
  /* Subsystems */
  private final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  private final Shooter s_PortShooter = new Shooter
    (
      () -> swerveState,
      ShooterConstants.portShooterOffset,
      IDConstants.portShooterIDs,
      ShooterConstants.TurretConstants.portPotOffset,
      true
    );
  private final Shooter s_StbdShooter = new Shooter
    (
      () -> swerveState,
      ShooterConstants.stbdShooterOffset,
      IDConstants.stbdShooterIDs,
      ShooterConstants.TurretConstants.stbdPotOffset,
      false
    );
  private final Vision s_Vision = new Vision
  (
    (poseEst, timestmp, stdDevs) -> 
    {
      s_Swerve.setVisionMeasurementStdDevs(stdDevs); 
      s_Swerve.addVisionMeasurement(poseEst, timestmp);
    },
    () -> swerveState.Speeds.omegaRadiansPerSecond,
    new Limelight(portLimelightName, VisionConstants.portLimelightOffset, s_PortShooter::getAzimuth, ShooterConstants.portShooterOffset), 
    new Limelight(stbdLimelightName, VisionConstants.stbdLimelightOffset, s_StbdShooter::getAzimuth, ShooterConstants.stbdShooterOffset)
  );
  private final LinearExtension s_Climber = new LinearExtension
  (
    IDConstants.climberCAN, 
    IDConstants.climberLimitDIO, 
    0, 
    ClimberConstants.maxPosition, 
    ClimberConstants.metersPerRotation,
    ClimberConstants.climberConfig
  );
  private final Hopper s_Hopper = new Hopper
  (
    IDConstants.spindexerCAN,
    IDConstants.intakeCAN, 
    IDConstants.extensionCAN, 
    IDConstants.extensionLimitDIO
  );
  private final VelocityMotor s_Feeder = new VelocityMotor
  (
    IDConstants.feederCAN,
    FeederConstants.feederConfig
  );
  
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

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

    if (!isSimulation()) {
      DataLogManager.start("/home/lvuser/logs");
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

    s_PortShooter.shootReadyTrigger()
      .and(s_StbdShooter.shootReadyTrigger())
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
    
    driver.x().and(s_Vision::hasLocalisation).onTrue
    (
      new PathFollowDrive
      (
        s_Swerve, 
        () -> this.swerveState,
        Pathfinding.testPath
      )
    );

    operator.povLeft().onTrue
    (
      run(() -> modifyTargets(target -> target.point = ControlConstants.leftFerryTarget.get()))
    );

    operator.povRight().onTrue
    (
      run(() -> modifyTargets(target -> target.point = ControlConstants.rightFerryTarget.get()))
    );

    // new Trigger(() -> FieldUtils.inAllianceZone(getTranslation()))
    //   .onTrue(run(() -> modifyTargets(target -> target.state = TargetState.Hub)))
    //   .onFalse(run(() -> modifyTargets(target -> target.state = TargetState.Point)));
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
    {swerveState = s_Swerve.getState();}

  /** Returns the t2d of the robot centre in field coordinates */
  public Translation2d getTranslation()
    {return swerveState.Pose.getTranslation();}

  /** Returns the r2d of the robot in field coordinates */
  public Rotation2d getRotation() 
    {return swerveState.Pose.getRotation();}
  
  /* OPMODE METHODS */
  /* ============ */
  @Override
  public void robotPeriodic() 
  {
    updateSwerveState();
    CommandScheduler.getInstance().run();
    PBDash.CAN_LOAD.put((double)canBus.getStatus().BusUtilization);
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
  public void autonomousInit() 
  {
    FieldUtils.updateAlliance();
    autoCommand = AutoFactories.getCommandList(PBDash.AUTO_STRING.get(), s_Swerve, () -> swerveState);

    if (autoCommand != null) CommandScheduler.getInstance().schedule(autoCommand);
  }

  @Override
  public void teleopInit() 
  {
    if (autoCommand != null) autoCommand.cancel();
    FieldUtils.updateAlliance();
    initInputTransmute();
  }

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic()
  {
    s_StbdShooter.setManual(PBDash.getDouble("Test Azimuth"), PBDash.getDouble("Test Altitude"));
    s_PortShooter.setManual(PBDash.getDouble("Test Azimuth"), PBDash.getDouble("Test Altitude"));
    
    s_StbdShooter.setFlySpeed(PBDash.getDouble("Test Flyspeed"));
    s_PortShooter.setFlySpeed(PBDash.getDouble("Test Flyspeed"));
  }
}