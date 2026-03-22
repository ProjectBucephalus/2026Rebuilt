// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.constants.*;
import frc.robot.constants.Constants.*;
import frc.robot.constants.FieldConstants.GeoFencing;

import frc.robot.subsystems.*;
import frc.robot.subsystems.generic.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Target.TargetState;
import frc.robot.subsystems.vision.*;

import frc.robot.util.*;
import frc.robot.util.autobuilder.AutoBuilder;
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
  public static enum ClimbPosition { Left, Right }

  @Logged
  public static class RobotState 
  {
    public ClimbPosition climbPos = ClimbPosition.Left;
    public boolean nudging = true;
  }
  
  @Logged
  private final RobotState state = new RobotState();

  private SwerveDriveState swerveState = new SwerveDriveState();
  private Optional<Command> autoCommand = Optional.empty();

  /* Telemetry and SD */
  private final Telemetry ctreLogger = new Telemetry(SwerveConstants.maxSpeed);
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
    () -> swerveState,
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
    () -> swerveState,
    ShooterConstants.stbdShooterOffset,
    IDConstants.stbdShooterIDs,
    ShooterConstants.TurretConstants.stbdPotOffset,
    false,
    ShooterConstants.HoodConstants.stbdHomeAngle,
    false
  );
  
  @Logged(name = "Vision")
  private final Vision s_Vision = new Vision
  (
    s_Swerve::addVisionMeasurement,
    () -> swerveState.Speeds.omegaRadiansPerSecond,
    new Limelight(IDConstants.portLimelightName, VisionConstants.portLimelightOffset, s_PortShooter::getAzimuthTimestamped, ShooterConstants.portShooterOffset), 
    new Limelight(IDConstants.stbdLimelightName, VisionConstants.stbdLimelightOffset, s_StbdShooter::getAzimuthTimestamped, ShooterConstants.stbdShooterOffset)
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
  
  @Logged(name = "Intake")
  private final Intake s_Intake = new Intake
  (
    IDConstants.intakeCAN, 
    IDConstants.extensionCAN
  );

  /* Rumble */
  private final RumbleRequester io_driverRight = new RumbleRequester(driver, RumbleType.kRightRumble, PBDash.RUMBLE_DRIVER::get);
  private final RumbleRequester io_driverLeft  = new RumbleRequester(driver, RumbleType.kLeftRumble, PBDash.RUMBLE_DRIVER::get);
  @SuppressWarnings("unused")
  private final RumbleRequester io_debugRight  = new RumbleRequester(operator, RumbleType.kRightRumble, PBDash.RUMBLE_OPERATOR::get);
  @SuppressWarnings("unused")
  private final RumbleRequester io_debugLeft   = new RumbleRequester(operator, RumbleType.kLeftRumble, PBDash.RUMBLE_OPERATOR::get);
  
  /* Input Transmutation */
  private final JoystickTransmuter driverStick = new JoystickTransmuter(driver::getLeftY, driver::getLeftX).invertX().invertY();
  private final Brake driverBrake = new Brake(() -> Math.max(driver.getRightTriggerAxis(), s_Intake.brakeFromIntake()), ControlConstants.maxThrottle, ControlConstants.minThrottle);
  private final InputCurve driverInputCurve = new InputCurve(2);
  private final Deadband driverDeadband = new Deadband();

  public Robot() 
  {
    updateSwerveState();
    
    initLogging();
    initInputTransmute();

    new ControlBinder
    (
      state, 
      () -> swerveState, 
      driver, 
      operator, 
      switchboard, 
      driverStick, 
      driverBrake, 
      s_Swerve, 
      s_Vision, 
      s_PortShooter, 
      s_StbdShooter, 
      s_Intake, 
      s_Climber
    )
    .bind();

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

    PBDash.putSendable("Current Commands", CommandScheduler.getInstance());
  }

  /** Set up input modification and fencing systems */
  private void initInputTransmute()
  {
    driverStick
      .rotated(FieldUtils.isAlliance(Alliance.Red))
      .withFieldObjects(GeoFencing.fieldGeoFence)
      .withBrake(driverBrake)
      .withInputCurve(driverInputCurve)
      .withDeadband(driverDeadband);

    FieldObject.setRobotRadiusSup(() -> SwerveConstants.robotRadiusExpanded);
    FieldObject.setRobotPosSup(() -> swerveState.Pose.getTranslation());

    GeoFencing.fieldGeoFence.setActiveCondition(() -> s_Vision.hasLocalisation() && PBDash.IO_FENCE.get());

    GeoFencing.fieldRedGeoFence.setActiveCondition(() -> FieldUtils.isAlliance(Alliance.Red));
    GeoFencing.fieldBlueGeoFence.setActiveCondition(() -> FieldUtils.isAlliance(Alliance.Blue));

 /*   GeoFencing.hubBlueOutput
      .setActiveCondition
      (
        () -> 
        FieldUtils.hubActiveToleranced
        (
          Alliance.Blue, 
          ControlConstants.preShiftOutputMargin, 
          ControlConstants.postShiftOutputMargin
        ) && !state.nudging
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
        ) && !state.nudging
      ); */
  }

  /** Sets trigger conditions to activate controller rumbles */
  private void bindRumbles()
  {
    new Trigger(() -> FieldUtils.hubActive(FieldUtils.getAlliance())) 
      .onChange(io_driverLeft.timedRequestCommand("Shift Change", 0.5));
  }

  /* UTIL METHODS */
  /* ============ */

  /** Pull current state from drivebase for external use, to avoid repeated expensive calls */
  private void updateSwerveState()
  {
    swerveState = s_Swerve.getState();
    PBDash.FIELD.setRobotPose(swerveState.Pose);
  }

  private void compileAuto()
  {
    autoCommand = Optional.of(AutoBuilder.compile(PBDash.AUTO_STRING.get(), () -> swerveState, s_Swerve, s_Intake));
  }

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
    PBDash.updateSendables();
    updateSwerveState();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit()
  {
    FieldUtils.updateAlliance();
    if (swerveState.Pose.getTranslation().equals(Translation2d.kZero))
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
    
    //if (autoCommand.isEmpty())
      compileAuto();

    CommandScheduler.getInstance().schedule(autoCommand.get());
  }

  @Override
  public void teleopInit() 
  {
    MatchTime.startTele();
    FieldUtils.updateAlliance();
    
    autoCommand.ifPresent(Command::cancel);

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
      s_PortShooter.target.state = TargetState.Manual;
      s_PortShooter.target.altitude = PBDash.TEST_ALTITUDE.get();
      s_PortShooter.target.speed = PBDash.TEST_FLYSPEED.get();
      s_StbdShooter.target.state = TargetState.Manual;
      s_StbdShooter.target.altitude = PBDash.TEST_ALTITUDE.get();
      s_StbdShooter.target.speed = PBDash.TEST_FLYSPEED.get();
    }
  }
}