// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpringMotor extends SubsystemBase 
{
  private final TalonFX m_Spring;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  private double target;

  /** Creates a new SpringMotor. */
  public SpringMotor(int id, TalonFXConfiguration config) 
  {
    m_Spring = new TalonFX(id);
    m_Spring.getConfigurator().apply(config);
  }

    /**
   * Sets the target point for the motor 
   * @param target mechanism rotations
   */
  public void setTarget(double pos)
    {target = pos;}

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

  // /**
  //  * Creates a command to continuously adjust the target point of the motor by a dynamic amount <p>
  //  * Primarily intended for joystick control
  //  * @param shiftSup A supplier for the amount to adjust the target by in mechanism rotations
  //  * @return the Command
  //  */
  // public Command adjustTargetCmd(DoubleSupplier shiftSup) 
  //   {return run(() -> {if (shiftSup.getAsDouble() != 0) baseSetTarget(getAngle() + shiftSup.getAsDouble());});}

  /** @return Current angle of the motor, in mechanism rotations */
  @Logged(name = "angle Rotations")
  public double getAngle() 
    {return m_Spring.getPosition().getValue().in(Units.Rotations);}

  @Override
  public void periodic() 
  {
    // Slot 1 is spring, slot 0 is hardstop
    int slot = getAngle() >= target ? 1 : 0;
    m_Spring.setControl(request.withPosition(target).withSlot(slot));
  }
}
