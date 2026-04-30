package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class PositionMotor extends SubsystemBase
{
  protected final TalonFX m_Position;

  protected final MotionMagicVoltage request = new MotionMagicVoltage(0);

  protected boolean active = false;

  private final double motorGearRatio;
  private final DCMotorSim motorSim;

  /**
   * Creates a wrapper around a TalonFX to provide velocity control
   * 
   * @param id the id of the motor
   * @param config the config to apply to the wrapped motor
   */
  public PositionMotor(int id, TalonFXConfiguration config) 
  {
    m_Position = new TalonFX(id);

    motorGearRatio = config.Feedback.SensorToMechanismRatio;
    motorSim = new DCMotorSim
    (
      LinearSystemId.createDCMotorSystem
        (DCMotor.getKrakenX60(1), 0.1, motorGearRatio * config.Feedback.RotorToSensorRatio),
      DCMotor.getKrakenX60(1)
    );

    if (RobotBase.isSimulation())
    {
      config = config.clone();
      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    }

    m_Position.getConfigurator().apply(config);
  }

  /**
   * Sets the target point for the motor 
   * @param target mechanism rotations
   */
  public void setTarget(double pos)
    {baseSetTarget(pos);}

  /**
   * Sets the target point for the motor, can't be overridden
   * Exists as a sort of hacky solution to get around overriden versions of the method
   * @param target mechanism rotations
   */
  protected final void baseSetTarget(double pos)
  {
    active = true;
    m_Position.setControl(request.withPosition(pos));
  }

  /**
   * Creates a command to set the target point for the motor <p>
   * @param targetSup mechanism rotations
   * @return the Command
   */
  public Command setTargetCmd(DoubleSupplier targetSup)
    {return runOnce(() -> setTarget(targetSup.getAsDouble()));}

  /**
   * Creates a command to set the target point for the motor <p>
   * NOTE: The provided value is only evaluated when the command is created
   * @param target mechanism rotations
   * @return the Command
   */
  public Command setTargetCmd(double target)
    {return setTargetCmd(() -> target);}

  /**
   * Creates a command to set the target point for the motor and wait until we reach that target point <p>
   * @param targetSup mechanism rotations
   * @return the Command
   */
  public Command gotoTargetCmd(DoubleSupplier targetSup)
    {return setTargetCmd(targetSup).andThen(Commands.waitUntil(this::atTarget));}

    /**
   * Creates a command to set the target point for the motor and wait until we reach that target point <p>
   * NOTE: The provided value is only evaluated when the command is created
   * @param target mechanism rotations
   * @return the Command
   */
  public Command gotoTargetCmd(double target)
    {return gotoTargetCmd(() -> target);}

  /**
   * Creates a command to continuously adjust the target point of the motor by a dynamic amount <p>
   * Primarily intended for joystick control
   * @param shiftSup A supplier for the amount to adjust the target by in mechanism rotations
   * @return the Command
   */
  public Command adjustTargetCmd(DoubleSupplier shiftSup) 
    {return run(() -> {if (shiftSup.getAsDouble() != 0) baseSetTarget(getAngle() + shiftSup.getAsDouble());});}
  
  /** @return Current angle of the motor, in mechanism rotations */
  public double getPosition()
    {return getAngle();}

  /** @return Current angle of the motor, in mechanism rotations */
  @Logged(name = "angle Rotations")
  public double getAngle() 
    {return m_Position.getPosition().getValue().in(Units.Rotations);}

  /** @return Current angle of the motor, in mechanism rotations */
  @Logged(name = "target Rotations")
  public double getTarget() 
    {return request.Position;}

  /** Stops the motor */
  public void stop()
    {m_Position.set(0);}

  /** @return {@code true} when the motor is close to target */
  public boolean atTarget()
    {return MathUtil.isNear(getTarget(), getPosition(), 0.1);}

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
    {return m_Position.isConnected();}

  @Override
  public void simulationPeriodic()
  {
    var motorSimState = m_Position.getSimState();
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
