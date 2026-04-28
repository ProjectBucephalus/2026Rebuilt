package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * A subsystem wrapped around a TalonFX to provide a simple subsystem for controlling a motor with mechanism ratio
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class VelocityMotor extends SubsystemBase 
{
  protected final TalonFX m_Velocity;

  protected final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
  
  private final DCMotorSim motorSim;
  private final double motorGearRatio;

  /**
   * Creates a wrapper around a TalonFX to provide velocity control
   * 
   * @param id the id of the motor
   * @param config the config to apply to the wrapped motor
   */
  public VelocityMotor(int id, TalonFXConfiguration config) 
  {
    m_Velocity = new TalonFX(id);
    m_Velocity.getConfigurator().apply(config);

    motorGearRatio = config.Feedback.SensorToMechanismRatio;
    motorSim = new DCMotorSim
    (
      LinearSystemId.createDCMotorSystem
        (DCMotor.getKrakenX60(1), 0.01, config.Feedback.SensorToMechanismRatio),
      DCMotor.getKrakenX60(1)
    );
  }

  /**
   * Set the speed of the motor
   * 
   * @param speed the desired speed, in mechanism rotations per second
   */
  public void setSpeed(double speed)
    {m_Velocity.setControl(request.withVelocity(speed));}

  /**
   * Construct a command that sets the speed of the motor <p>
   * 
   * @param speedSup A supplier providing the desired speed, in mechanism rotations per second
   * @return the {@link Command}
   */
  public Command setSpeedCmd(DoubleSupplier speedSup)
    {return runOnce(() -> setSpeed(speedSup.getAsDouble()));}

  public Command runCmd(DoubleSupplier speedSup)
    {return runEnd(() -> setSpeed(speedSup.getAsDouble()), () -> setSpeed(0));}

  /** @return Current speed of the motor, in mechanism rotations per second */
  @Logged(name = "speed RevPerSec")
  public double getSpeed() 
    {return m_Velocity.getVelocity().getValue().in(Units.RotationsPerSecond);}

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
    {return m_Velocity.isConnected();}

  @Override
  public void simulationPeriodic()
  {
    var motorSimState = m_Velocity.getSimState();
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = motorSimState.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSim.setInputVoltage(motorVoltage.in(Units.Volts));
    motorSim.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    motorSimState.setRawRotorPosition(motorSim.getAngularPosition().times(motorGearRatio));
    motorSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorGearRatio));
  }
}
