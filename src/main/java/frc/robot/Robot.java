// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.commands.swerve.*;
import frc.robot.constants.*;
import static frc.robot.constants.IDConstants.*;
import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.FieldConstants.GeoFencing.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.Vision.TagPOI;
import frc.robot.util.AutoFactories;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;
import frc.robot.util.controlTransmutation.*;
import frc.robot.util.libs.Telemetry;

@Logged
public class Robot extends TimedRobot 
{
  /* Enums */
  public enum TargetPosition {Left, Right, Centre, None}
  public enum DriveState {None, Hub, Tower}
  
  /* State */
  private SwerveDriveState swerveState;
  private TargetPosition currentTarget = TargetPosition.None;
  private DriveState currentDriveState = DriveState.None;
  private Command autoCommand;

  /* Telemetry and SD */
  private Field2d field = new Field2d();
  private final Telemetry ctreLogger = new Telemetry(Constants.Swerve.maxSpeed);
  
  /* Subsystems */
  private final static CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  private final Shooter s_Shooter = new Shooter
    (
      () -> swerveState.Pose,
      IDConstants.shooterMainID, 
      IDConstants.shooterAuxID, 
      IDConstants.turretID, 
      IDConstants.hoodID
    );
  private final Vision s_Vision = new Vision
    (
      (poseEst, timestmp, stdDevs) -> 
      {
        s_Swerve.setVisionMeasurementStdDevs(stdDevs); 
        s_Swerve.addVisionMeasurement(poseEst, timestmp);
      },
      () -> Pair.of(s_Swerve.getPigeon2().getYaw().getValueAsDouble(), swerveState.Speeds.omegaRadiansPerSecond), 
      new Limelight(foreLimelightName), 
      new Limelight(aftLimelightName)
    );

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Rumble */
  private final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, SD.RUMBLE_DRIVER);
  private final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, SD.RUMBLE_DRIVER);
  private final RumbleRequester io_operatorRight  = new RumbleRequester(operator, RumbleType.kRightRumble, SD.RUMBLE_OPERATOR);
  private final RumbleRequester io_operatorLeft   = new RumbleRequester(operator, RumbleType.kLeftRumble, SD.RUMBLE_OPERATOR);
  
  /* Input Transmutation */
  private final JoystickTransmuter driverStick = new JoystickTransmuter(driver::getLeftY, driver::getLeftX).invertX().invertY();
  private final Brake driverBrake = new Brake(driver::getRightTriggerAxis, Constants.Control.maxThrottle, Constants.Control.minThrottle);
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
  private void initLogging() 
  {
    SignalLogger.enableAutoLogging(false);

    if (!isSimulation()) {
      DataLogManager.start("/home/lvuser/logs");
      DriverStation.startDataLog(DataLogManager.getLog());
    }

    //Epilogue.bind(this);

    SmartDashboard.putData("Field", field);

    s_Swerve.registerTelemetry(ctreLogger::telemeterize);
  }

  private void initInputTransmute()
  {
    boolean redAlliance = FieldUtils.isRedAlliance();
    
    driverStick
      .rotated(redAlliance)
      .withFieldObjects(GeoFencing.fieldGeoFence)
      .withBrake(driverBrake)
      .withInputCurve(driverInputCurve)
      .withDeadband(driverDeadband);

    FieldUtils.activateAllianceFencing(redAlliance);
    FieldConstants.GeoFencing.configureAttractors((testTarget, testState) -> currentTarget == testTarget && currentDriveState == testState);
    FieldObject.setRobotRadiusSup
      (() -> 
        Math.hypot(swerveState.Speeds.vxMetersPerSecond, swerveState.Speeds.vyMetersPerSecond) >= robotSpeedThreshold ? 
        robotRadiusCircumscribed : 
        robotRadiusInscribed
      );
    FieldObject.setRobotPosSup(this::getTranslation);
    GeoFencing.fieldGeoFence.setActiveCondition(SD.FENCE_TOGGLE::get);
  }

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
        1, 
        Rotation2d.k180deg,
        new Translation2d[] 
        {
          new Translation2d(15, 2),
          new Translation2d(11, 2),
          new Translation2d(11, 6)
        }
      )
    );

    /* Heading Locking */
    new Trigger(() -> currentDriveState == DriveState.None)
      .onTrue(runOnce(() -> s_Vision.setActivePOI(TagPOI.ALL)))
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
    driver.start().onTrue(runOnce(s_Vision::resetRotation).ignoringDisable(true));
    new Trigger(SD.LL_EXPOSURE_UP::button).onTrue(runOnce(s_Vision::incrementPipeline));
    new Trigger(SD.LL_EXPOSURE_DOWN::button).onTrue(runOnce(s_Vision::decrementPipeline));
  }

  private void bindRumbles()
  {
    io_operatorRight.addRumbleTrigger("ScoreReady", new Trigger(() -> false)); // EXAMPLE
  }

  /* UTIL METHODS */
  /* ============ */
  public static void setYaw(double newYaw) {s_Swerve.getPigeon2().setYaw(newYaw);}

  private void updateSwerveState()
  {
    swerveState = s_Swerve.getState();
    field.setRobotPose(swerveState.Pose);
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
    if (getTranslation().equals(Translation2d.kZero))
    {
      s_Swerve.resetPose(FieldUtils.isRedAlliance() ? FieldConstants.redStartLine : FieldConstants.blueStartLine);
    }
  }

  @Override
  public void autonomousInit() 
  {
    autoCommand = AutoFactories.getCommandList(SD.AUTO_STRING.get(), s_Swerve, () -> swerveState);

    if (autoCommand != null) CommandScheduler.getInstance().schedule(autoCommand);
  }

  @Override
  public void teleopInit() 
  {
    if (autoCommand != null) autoCommand.cancel();
  }

  @Override
  public void testInit() {CommandScheduler.getInstance().cancelAll();}
}