package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.constants.Constants.ShooterConstants.FlywheelConstants.*;

import frc.robot.Robot;
import frc.robot.constants.Constants.ShooterConstants;

/**
 * Interface class for a shooter flywheel. <p>
 * Uses two linked TalonFX controlled motors. <p>
 * Uses MotionMagic to control velocity.
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Flywheels 
{
  private final TalonFX m_Leader; 
  private final TalonFX m_Follower;

  private boolean speedCheck = false;

  private final DCMotorSim motorSim = new DCMotorSim
  (
    LinearSystemId.createDCMotorSystem
      (DCMotor.getKrakenX60(2), 0.04, mainWheelBeltRatio),
    DCMotor.getKrakenX60(2)
  );

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

  /**
   * Creates a velocity controlled flywheel, to be managed by {@link ShooterConstants} master-system
   * @param leaderCAN CAN-ID of primary shooter motor
   * @param followerCAN CAN-ID of secondary shooter motor, set to follow first
   */
  public Flywheels(int leaderCAN, int followerCAN)
  {
    m_Leader = new TalonFX(leaderCAN);
    m_Follower = new TalonFX(followerCAN);

    m_Leader.getConfigurator().apply(flywheelConfig);
    m_Follower.getConfigurator().apply(flywheelConfig);

    m_Follower.setControl(new Follower(leaderCAN, MotorAlignmentValue.Opposed));
  }

  /**
   * Set the motor speed
   * 
   * @param speed the desired speed, in mechanism rotations per second
   */
  public void setSpeed(double speed)
    {m_Leader.setControl(request.withVelocity(speed));}

  /**
   * Checks if the current motor speed is within {@link ShooterConstants#flySpeedTolerance flySpeedTolerance} of the requested speed, using a Schmitt trigger
   * 
   * @return true if the motor is at speed
   */
  @Logged
  public boolean atSpeed() 
  {
    // Use given tolerance for reaching speed, use double tolerance for no longer being at speed
    if (MathUtil.isNear(Math.max(request.Velocity, idleSpeed), getSpeed(), flySpeedTolerance))
      speedCheck = true;
    else if (!MathUtil.isNear(Math.max(request.Velocity, idleSpeed), getSpeed(), flySpeedTolerance * 2))
      speedCheck = false;

    return speedCheck;
  }

  /** @return Current speed of the flywheels (RPS of the main flywheel) */
  @Logged(name = "speed RevPerSec")
  public double getSpeed() 
  {
    if (Robot.isSimulation())
      return motorSim.getAngularVelocity().in(Units.RotationsPerSecond);
    else 
      return m_Leader.getVelocity().getValue().in(Units.RotationsPerSecond);
  }

  @Logged(name = "temp Celsius")
  public double getTemp() 
  {
    if (Robot.isSimulation())
      return -1; 
    else 
      return m_Leader.getAncillaryDeviceTemp().getValue().in(Units.Celsius);
  }

  @Logged(name = "current draw Amps")
  public double getMotorCurrent()
  {
    if (Robot.isSimulation())
      return motorSim.getCurrentDrawAmps();
    else 
      return m_Leader.getStatorCurrent().getValue().in(Units.Amps);
  }

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
    {return m_Leader.isConnected() && m_Follower.isConnected();}

  public void update()
  {

  }

  protected void updateSim()
  {
    var leaderSimState = m_Leader.getSimState();
    var followerSimState = m_Leader.getSimState();
    leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the average voltage of the motors
    var motorVoltage = (leaderSimState.getMotorVoltage() + followerSimState.getMotorVoltage()) / 2;

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSim.setInputVoltage(motorVoltage);
    motorSim.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    leaderSimState.setRawRotorPosition(motorSim.getAngularPosition().times(mainWheelBeltRatio));
    leaderSimState.setRotorVelocity(motorSim.getAngularVelocity().times(mainWheelBeltRatio));
    followerSimState.setRawRotorPosition(motorSim.getAngularPosition().times(mainWheelBeltRatio));
    followerSimState.setRotorVelocity(motorSim.getAngularVelocity().times(mainWheelBeltRatio));
  }
}
