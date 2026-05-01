package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.Constants.SwerveConstants.driveKD;
import static frc.robot.constants.Constants.SwerveConstants.driveKI;
import static frc.robot.constants.Constants.SwerveConstants.driveKP;
import static frc.robot.constants.Constants.SwerveConstants.maxAngularVelocity;
import static frc.robot.constants.Constants.SwerveConstants.maxSpeed;
import static frc.robot.constants.Constants.SwerveConstants.rotationKD;
import static frc.robot.constants.Constants.SwerveConstants.rotationKI;
import static frc.robot.constants.Constants.SwerveConstants.rotationKP;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.Conversions;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 * @author CTRE
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Sendable 
{
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  @SuppressWarnings("unused")
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine
  (
    new SysIdRoutine.Config
    (
      null,        // Use default ramp rate (1 V/s)
      Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
      null,        // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    ),
    new SysIdRoutine.Mechanism
    (
      output -> setControl(m_translationCharacterization.withVolts(output)),
      null,
      this
    )
  );

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  @SuppressWarnings("unused")
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine
  (
    new SysIdRoutine.Config
    (
      null,        // Use default ramp rate (1 V/s)
      Volts.of(7), // Use dynamic voltage of 7 V
      null,        // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    ),
    new SysIdRoutine.Mechanism
    (
      volts -> setControl(m_steerCharacterization.withVolts(volts)),
      null,
      this
    )
  );

  /*
    * SysId routine for characterizing rotation.
    * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
    * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
    */
  @SuppressWarnings("unused")
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine
  (
    new SysIdRoutine.Config
    (
      /* This is in radians per second², but SysId only supports "volts per second" */
      Volts.of(Math.PI / 6).per(Second),
      /* This is in radians per second, but SysId only supports "volts" */
      Volts.of(Math.PI),
      null, // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    ),
    new SysIdRoutine.Mechanism
    (
      output -> 
      {
        /* output is actually radians per second, but SysId only supports "volts" */
        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
        /* also log the requested output for SysId */
        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
      },
      null,
      this
    )
  );

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
    * <p>
    * This constructs the underlying hardware devices, so users should not construct
    * the devices themselves. If they need the devices, they can access them through
    * getters in the classes.
    *
    * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
    * @param modules             Constants for each specific module
    */
  public CommandSwerveDrivetrain
  (
    SwerveDrivetrainConstants drivetrainConstants,
    SwerveModuleConstants<?, ?, ?>... modules
  ) 
  {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) 
      {startSimThread();}
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
    * <p>
    * This constructs the underlying hardware devices, so users should not construct
    * the devices themselves. If they need the devices, they can access them through
    * getters in the classes.
    *
    * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
    * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
    *                                   unspecified or set to 0 Hz, this is 250 Hz on
    *                                   CAN FD, and 100 Hz on CAN 2.0.
    * @param modules                    Constants for each specific module
    */
  public CommandSwerveDrivetrain
  (
    SwerveDrivetrainConstants drivetrainConstants,
    double odometryUpdateFrequency,
    SwerveModuleConstants<?, ?, ?>... modules
  ) 
  {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) 
      {startSimThread();}
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
    * <p>
    * This constructs the underlying hardware devices, so users should not construct
    * the devices themselves. If they need the devices, they can access them through
    * getters in the classes.
    *
    * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
    * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
    *                                   unspecified or set to 0 Hz, this is 250 Hz on
    *                                   CAN FD, and 100 Hz on CAN 2.0.
    * @param odometryStandardDeviation  The standard deviation for odometry calculation
    *                                  in the form [x, y, theta]ᵀ, with units in meters
    *                                  and radians
    * @param visionStandardDeviation   The standard deviation for vision calculation
    *                                  in the form [x, y, theta]ᵀ, with units in meters
    *                                  and radians
    * @param modules                    Constants for each specific module
    */
  public CommandSwerveDrivetrain
  (
    SwerveDrivetrainConstants drivetrainConstants,
    double odometryUpdateFrequency,
    Matrix<N3, N1> odometryStandardDeviation,
    Matrix<N3, N1> visionStandardDeviation,
    SwerveModuleConstants<?, ?, ?>... modules
  ) 
  {
    super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    if (Utils.isSimulation()) 
      {startSimThread();}
  }

  /** @author 5985, based on external docs */
  @Override
  public void initSendable(SendableBuilder builder) 
  {
    builder.setSmartDashboardType("Subsystem");

    builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
    builder.addStringProperty
    (
      ".default",
      () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
      null
    );
    builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
    builder.addStringProperty
    (
      ".command",
      () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
      null
    );
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
    *
    * @param request Function returning the request to apply
    * @return Command to run
    */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) 
    {return run(() -> this.setControl(requestSupplier.get()));}

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
    * specified by {@link #m_sysIdRoutineToApply}.
    *
    * @param direction Direction of the SysId Quasistatic test
    * @return Command to run
    */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) 
    {return m_sysIdRoutineToApply.quasistatic(direction);}

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
    * specified by {@link #m_sysIdRoutineToApply}.
    *
    * @param direction Direction of the SysId Dynamic test
    * @return Command to run
    */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) 
    {return m_sysIdRoutineToApply.dynamic(direction);}

  /**
   * Calculates the required drive input to drive to a pose using a PID control loop, accounting for geofencing
   * 
   * @param target Current target pose to drive towards
   * @param pose Current robot pose
   * @param brake Throttle to apply, [0..1]. 1 is full speed, 0 is stopped
   * @return Chassis speeds, m/s, m/s, rad/s
   */
  public ChassisSpeeds calculateDrivePID(Pose2d target, Pose2d pose, double translationThrottle, double rotationThrottle)
  {
    final PIDController xController = new PIDController(driveKP, driveKI, driveKD);
    final PIDController yController = new PIDController(driveKP, driveKI, driveKD);
    final PIDController thetaController = new PIDController(rotationKP, rotationKI, rotationKD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    final var robotPos = pose.getTranslation();
    final var targetPos = target.getTranslation();

    double speedX = xController.calculate(robotPos.getX(), targetPos.getX());
    double speedY = yController.calculate(robotPos.getY(), targetPos.getY());
    double throttleX;
    double throttleY;
    
    if(Math.abs(speedX) > Math.abs(speedY))
    {
      double ratio = (speedX==0 || speedY==0) ? 0 : speedY/speedX;
      throttleX = Conversions.clamp(speedX);
      throttleY = throttleX*ratio;
    }
    else
    {
      double ratio = (speedX==0 || speedY==0) ? 0 : speedX/speedY;
      throttleY = Conversions.clamp(speedY);
      throttleX = throttleY*ratio;
    }
    final var throttleXY = FieldConstants.GeoFencing.fieldGeoFence.process(new Translation2d(throttleX, throttleY).times(translationThrottle));
    
    final double speedTheta = 
      Conversions.clamp
      (
        thetaController.calculate(pose.getRotation().getRadians(), target.getRotation().getRadians()), 
        -maxAngularVelocity, 
        maxAngularVelocity
      ) * rotationThrottle;

    xController.close();
    yController.close();
    thetaController.close();

    return ChassisSpeeds.fromFieldRelativeSpeeds
    (
      new ChassisSpeeds
      (
        throttleXY.getX() * maxSpeed,
        throttleXY.getY() * maxSpeed,
        speedTheta
      ),
      pose.getRotation()
    );
  }

  @Override
  public void periodic()
  {
    /*
      * Periodically try to apply the operator perspective.
      * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
      * This allows us to correct the perspective in case the robot code restarts mid-match.
      * Otherwise, only check and apply the operator perspective if the DS is disabled.
      * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
      */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) 
    {
      DriverStation.getAlliance().ifPresent
      (allianceColor -> 
        {
          setOperatorPerspectiveForward
          (
            allianceColor == Alliance.Red
              ? kRedAlliancePerspectiveRotation
              : kBlueAlliancePerspectiveRotation
          );
          m_hasAppliedOperatorPerspective = true;
        }
      );
    }
  }

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
  {
    return moduleValid(0)
        && moduleValid(1)
        && moduleValid(2)
        && moduleValid(3)
        && getPigeon2().isConnected();
  }

  private boolean moduleValid(int index)
  {
    var module = getModule(index);
    return module.getEncoder().isConnected()
        && module.getDriveMotor().isConnected()
        && module.getSteerMotor().isConnected();
  }

  private void startSimThread() 
  {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier
    (
      () -> 
      {
        final double currentTime = Utils.getCurrentTimeSeconds();
        double deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;
    
        /* use the measured time delta, get battery voltage from WPILib */
        updateSimState(deltaTime, RobotController.getBatteryVoltage());
      }
    );
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
