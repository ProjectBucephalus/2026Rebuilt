  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.subsystems;

  import com.ctre.phoenix6.configs.TalonFXConfiguration;
  import com.ctre.phoenix6.controls.Follower;
  import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
  import com.ctre.phoenix6.hardware.TalonFX;
  import com.ctre.phoenix6.signals.MotorAlignmentValue;

  import edu.wpi.first.math.MathUtil;
  import edu.wpi.first.math.system.plant.DCMotor;
  import edu.wpi.first.math.system.plant.LinearSystemId;
  import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
  import edu.wpi.first.networktables.NetworkTableInstance;
  import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;
  import frc.robot.constants.IDConstants;

  import static frc.robot.constants.Constants.Shooter.*;

  public class Shooter extends SubsystemBase {
    private TalonFX m_Shooter1;
    private TalonFX m_Shooter2;
    private final Follower m_Follower = new Follower(IDConstants.shooter1ID, MotorAlignmentValue.Opposed);
    
    final MotionMagicVelocityDutyCycle m_Request = new MotionMagicVelocityDutyCycle(0);
    private final NetworkTableEntry simRpsEntry =
      NetworkTableInstance.getDefault()
          .getTable("Telemetry")
          .getEntry("SimMotorRPS");

    private State state;
    
    public enum State
      {
        IDLE,
        REV,
        SHOOT_SING,
        SHOOT_MULT,
        TESTING
      };
    private int shotsQueued;
    private boolean indexing;
    /** Creates a new shooter. */
    public Shooter() 
    {
      state = State.TESTING;
      SmartDashboard.putNumber("topSpeed", 0.0);
      SmartDashboard.putNumber("bottomSpeed", 0.0);
      m_Shooter1 = new TalonFX(IDConstants.shooter1ID);
      m_Shooter2 = new TalonFX(IDConstants.shooter2ID);

      m_Shooter2.setControl(m_Follower);
      // in init function
      var shooterConfigs = new TalonFXConfiguration();

      // set slot 0 gains
      shooterConfigs.Slot0.kS = slot0S; 
      shooterConfigs.Slot0.kV = slot0V; 
      shooterConfigs.Slot0.kA = slot0A; 
      shooterConfigs.Slot0.kP = slot0P;
      shooterConfigs.Slot0.kI = slot0I; 
      shooterConfigs.Slot0.kD = slot0D; 

      // set Motion Magic settings
      shooterConfigs.MotionMagic.MotionMagicCruiseVelocity = velocity; 
      shooterConfigs.MotionMagic.MotionMagicAcceleration = acceleration;


      m_Shooter1.getConfigurator().apply(shooterConfigs);
      m_Shooter2.getConfigurator().apply(shooterConfigs);
    }

    public Command idle(double speed)
    {
      return this.run(() -> m_Shooter1.setControl(m_Request.withVelocity(speed)));
    }

    public Command shoot(double targetSpeed)
    {
      return this.run(() -> m_Shooter1.setControl(m_Request.withVelocity(targetSpeed)));
    }

    public Command testMotor1(double speed)
    {
      return this.run(() -> m_Shooter1.setControl(m_Request.withVelocity(speed)));
    }

    public Command testMotor2(double speed)
    {
      return this.run(() -> m_Shooter2.setControl(m_Request.withVelocity(speed)));
    }

    @Override
    public void periodic() {
      switch(state)
      {
        case IDLE:
          m_Shooter1.setControl(m_Request.withVelocity(idleSpeed));
          break;
        case REV:
          m_Shooter1.setControl(m_Request.withVelocity(revSpeed));
          if (shotsQueued >= 1 && MathUtil.isNear(revSpeed, m_Shooter1.getVelocity().getValueAsDouble(), leliency))
          {
            //start indexing
            indexing = true;
            if (shotsQueued == 1)
            {
              state = State.SHOOT_SING;
            } else
            {
              state = State.SHOOT_MULT;
            }
          }
          break;
        case SHOOT_SING:
          if (!indexing)
          {
            shotsQueued = 0;
            state = State.IDLE;
          }
          break;
        case SHOOT_MULT:
          if (!indexing)
          {
            shotsQueued -= 1;
            state = State.REV;
          }
          break;
        case TESTING:
          testMotor1(SmartDashboard.getNumber("topSpeed", 0.0));
          testMotor2(SmartDashboard.getNumber("bottomSpeed", 0.0));
          break;
      }
    }


    private final FlywheelSim m_flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(2),
              kMOI,
              kGearRatio
          ),
          DCMotor.getKrakenX60Foc(2)
      );

    
    @Override
    public void simulationPeriodic() {
      var m_shooter1SimState = m_Shooter1.getSimState();
      var m_shooter2SimState = m_Shooter2.getSimState();

      double appliedVolts = m_shooter1SimState.getMotorVoltage();

      m_flywheelSim.setInputVoltage(appliedVolts);
      m_flywheelSim.update(0.02);

      double wheelRadPerSec = m_flywheelSim.getAngularVelocityRadPerSec();
      double rotorRps = (wheelRadPerSec / (2 * Math.PI)) * kGearRatio;

      m_shooter1SimState.setRotorVelocity(rotorRps);
      m_shooter2SimState.setRotorVelocity(rotorRps);
      simRpsEntry.setDouble(rotorRps);
    }
  }
