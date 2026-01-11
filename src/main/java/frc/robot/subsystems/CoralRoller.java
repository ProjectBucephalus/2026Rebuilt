package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class CoralRoller extends SubsystemBase
{
  private TalonFX m_Coral;
  private DigitalInput io_Sensor;

  private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getFalcon500(1), 0.001, 1
    ),
    DCMotor.getFalcon500(1)
  );

  public CoralRoller()
  {
    m_Coral = new TalonFX(IDConstants.coralRollerID);
    io_Sensor = new DigitalInput(IDConstants.coralSensorDIO);
  }

  public boolean getSensor()
    {return io_Sensor.get();}
   
  public Command setSpeedCommand(double speed)
    {return this.run(() -> m_Coral.set(speed));}

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() 
  {
    var coralSim = m_Coral.getSimState();

    // set the supply voltage of the TalonFX
    coralSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = coralSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(motorVoltage.in(Units.Volts));
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    coralSim.setRawRotorPosition(m_motorSimModel.getAngularPosition());
    coralSim.setRotorVelocity(m_motorSimModel.getAngularVelocity());

    SmartDashboard.putNumber("RollerTest", m_motorSimModel.getAngularVelocityRPM());
  }
}
