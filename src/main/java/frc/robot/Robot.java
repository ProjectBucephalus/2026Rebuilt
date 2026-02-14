// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


//import edu.wpi.first.epilogue.Epilogue;
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

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.commands.swerve.*;
import frc.robot.constants.*;
import static frc.robot.constants.Constants.*;
import frc.robot.constants.FieldConstants.GeoFencing;

import static frc.robot.constants.IDConstants.*;

import static frc.robot.constants.FieldConstants.GeoFencing.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Shooter;
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
  /* Enums */
  public enum TargetPosition {Left, Right, Centre, None} // TODO Unused
  public enum DriveState {None, Hub, Tower} // Depending on how we're doing climb lineup, we can probably remove both of these
  
  /* State */
  private SwerveDriveState swerveState;
  private TargetPosition currentTarget = TargetPosition.None;
  private DriveState currentDriveState = DriveState.None;
  private Command autoCommand;

  /* Telemetry and SD */
  private final Telemetry ctreLogger = new Telemetry(SwerveConstants.maxSpeed);
  
  /* Subsystems */
  private final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  private final Shooter s_PortShooter = new Shooter
    (
      () -> swerveState,
      ShooterConstants.portShooterOffset,
      IDConstants.portShooterIDs,
      ShooterConstants.TurretConstants.portPotOffset,
      false // TODO confirm
    );
  private final Shooter s_StbdShooter = new Shooter
    (
      () -> swerveState,
      ShooterConstants.stbdShooterOffset,
      IDConstants.stbdShooterIDs,
      ShooterConstants.TurretConstants.stbdPotOffset,
      true // TODO confirm
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
  private final BinaryMotor s_Feeder = new BinaryMotor
  (
    IDConstants.feederCAN,
    FeederConstants.feederSpeed,
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

    //Epilogue.bind(this);

    s_Swerve.registerTelemetry(ctreLogger::telemeterize);
  }

  /** Set up input modification and fencing systems */
  private void initInputTransmute()
  {
    FieldUtils.activateAllianceFencing();
    FieldConstants.GeoFencing.configureAttractors((testTarget, testState) -> currentTarget == testTarget && currentDriveState == testState);
    FieldObject.setRobotRadiusSup
      (() -> 
        Math.hypot(swerveState.Speeds.vxMetersPerSecond, swerveState.Speeds.vyMetersPerSecond) >= robotSpeedThreshold ? 
        robotRadiusCircumscribed : 
        robotRadiusInscribed
      );
    FieldObject.setRobotPosSup(this::getTranslation);
    
    driverStick
      .rotated(FieldUtils.isRedAlliance())
      .withFieldObjects(GeoFencing.fieldGeoFence)
      .withBrake(driverBrake)
      .withInputCurve(driverInputCurve)
      .withDeadband(driverDeadband);

    GeoFencing.fieldGeoFence.setActiveCondition(() -> PBDash.FENCE_TOGGLE.get() && PBDash.LL_TOGGLE.get());
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

    /* Setting Drive States */
    driver.povLeft().onTrue(runOnce(() -> currentTarget = TargetPosition.Left));
    driver.povRight().onTrue(runOnce(() -> currentTarget = TargetPosition.Right));
    driver.povUp().onTrue(runOnce(() -> currentTarget = TargetPosition.Centre));
    driver.povDown().onTrue(runOnce(() -> currentTarget = TargetPosition.None));
    
    driver.x().onTrue
    (
      new PathFollowDrive
      (
        s_Swerve, 
        () -> this.swerveState,
        Pathfinding.testPath
      )
    );

    /* Heading Locking */
    new Trigger(() -> currentDriveState == DriveState.None)
      .whileTrue
      (
        new ManualDrive
        (
          s_Swerve, 
          driverStick::stickOutput,
          () -> -driver.getRightX(),
          driver::getRightTriggerAxis
        )
      );
    
    /* Other */
    new Trigger(PBDash.LL_EXPOSURE_UP::button).onTrue(runOnce(s_Vision::incrementPipeline));
    new Trigger(PBDash.LL_EXPOSURE_DOWN::button).onTrue(runOnce(s_Vision::decrementPipeline));
  }

  /** Sets trigger conditions to activate controller rumbles */
  private void bindRumbles()
  {
    io_operatorRight.addRumbleTrigger("ScoreReady", new Trigger(() -> false)); // EXAMPLE
  }

  /* UTIL METHODS */
  /* ============ */
  /** Pull current state from drivebase for external use, to avoid repeated expensive calls */
  private void updateSwerveState()
  {
    swerveState = s_Swerve.getState();
  }

  /** Returns the t2d of the robot centre in field coordinates */
  public Translation2d getTranslation() {return swerveState.Pose.getTranslation();}

  /** Returns the r2d of the robot in field coordinates */
  public Rotation2d getRotation() {return swerveState.Pose.getRotation();}
  
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
  public void autonomousInit() 
  {
    FieldUtils.startAuto(System.currentTimeMillis());
    FieldUtils.updateAlliance();
    autoCommand = AutoFactories.getCommandList(PBDash.AUTO_STRING.get(), s_Swerve, () -> swerveState);

    if (autoCommand != null) CommandScheduler.getInstance().schedule(autoCommand);
  }

  @Override
  public void teleopInit() 
  {
    FieldUtils.startTele(System.currentTimeMillis());
    if (autoCommand != null) autoCommand.cancel();
    FieldUtils.updateAlliance();
    initInputTransmute();
  }

  @Override
  public void testInit() {CommandScheduler.getInstance().cancelAll();}
}