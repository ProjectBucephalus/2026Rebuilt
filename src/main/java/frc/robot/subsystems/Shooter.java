  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.subsystems;

  import com.ctre.phoenix6.configs.TalonFXConfiguration;
  import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
  import com.ctre.phoenix6.hardware.TalonFX;

  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

  import edu.wpi.first.wpilibj2.command.SubsystemBase;
  import frc.robot.constants.IDConstants;
  import frc.robot.util.SD;

  import static frc.robot.constants.Constants.Shooter.*;

  public class Shooter extends SubsystemBase {
    public enum State
    {
      IDLE,
      REV,
    };

    private final TalonFX m_Main = new TalonFX(IDConstants.shooterMainID);
    private final TalonFX m_Aux = new TalonFX(IDConstants.shooterAuxID);
    
    private final MotionMagicVelocityDutyCycle m_Request = new MotionMagicVelocityDutyCycle(0);

    private State state = State.IDLE;

    /** Creates a new shooter. */
    public Shooter() 
    {
      SmartDashboard.putNumber("topSpeed", 0.0);
      SmartDashboard.putNumber("bottomSpeed", 0.0);

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

      m_Main.getConfigurator().apply(shooterConfigs);
      m_Aux.getConfigurator().apply(shooterConfigs);
    }

    public void setState(State newState)
    {
      state = newState;
    }

    @Override
    public void periodic() 
    {
      switch(state)
      {
        case IDLE:
          m_Main.setControl(m_Request.withVelocity(idleSpeed));
          m_Aux.setControl(m_Request.withVelocity(idleSpeed));
          break;
        case REV:
          m_Main.setControl(m_Request.withVelocity(SD.BOTTOM_SHOOTER_SPEED.get()));
          m_Aux.setControl(m_Request.withVelocity(SD.TOP_SHOOTER_SPEED.get()));
          break;
      }
    }
  }
