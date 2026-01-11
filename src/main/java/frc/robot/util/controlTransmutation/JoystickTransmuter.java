// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

/** Full joystick processor, takes XY suppliers and runs multiple layers of transmutation to give an XY output */
public class JoystickTransmuter extends InputTransmuter
{
  private InputCurve inputCurve;
  private Deadband deadband;
  private Brake brake;
  private ObjectList fieldObjectList;

  private boolean invertX = false;
  private boolean invertY = false;
  private boolean rotateThroughput = false;

  private DoubleSupplier inputX;
  private DoubleSupplier inputY;

  public final Supplier<Translation2d> stickOutputSup;

  /**
   * Creates a new JoystickTransmuter with the given input axes and default/empty modifiers
   * @param inputX DoubleSupplier of the X/Translation input
   * @param inputY DoubleSupplier of the Y/Strafe input
   */
  public JoystickTransmuter(DoubleSupplier inputX, DoubleSupplier inputY)
  {
    this.inputX = inputX;
    this.inputY = inputY;

    inputCurve = new InputCurve();
    deadband = new Deadband();
    brake = new Brake();
    fieldObjectList = new ObjectList();

    stickOutputSup = this::stickOutput;
  }

  /**
   * Processes the value from the joystick suppliers and returns the final output
   * @return Translation2d of final processed stick output, [-1..1],[-1..1]
   */
  public Translation2d stickOutput()
  {
    double motionX = invertX ? -inputX.getAsDouble() : inputX.getAsDouble();
    double motionY = invertY ? -inputY.getAsDouble() : inputY.getAsDouble();
    return process(new Translation2d(motionX, motionY));
  }

  @Override
  public Translation2d process(Translation2d controlInput)
  {
    Translation2d motionXY = 
    brake.process
    (
      inputCurve.process
      (
        deadband.process
        (
          controlInput
        )
      )
    );

    if (rotateThroughput)
      {motionXY = motionXY.unaryMinus();}
    
    motionXY = fieldObjectList.process(motionXY);
    
    if (rotateThroughput)
      {motionXY = motionXY.unaryMinus();}
    
    return motionXY;
  }

  /**
   * Sets the input curve for the joystick to be processed through
   * @param inputCurve Any input curve object
   * @return The joystickTransmuter with the new input curve layer
   */
  public JoystickTransmuter withInputCurve(InputCurve inputCurve)
  {
    this.inputCurve = inputCurve;
    return this;
  }

  /**
   * Sets the deadband for the joystick to be processed through
   * @param deadband Any deadband object
   * @return The joystickTransmuter with the new deadband layer
   */
  public JoystickTransmuter withDeadband(Deadband deadband)
  {
    this.deadband = deadband;
    return this;
  }

  /**
   * Sets the brake for the joystick to be processed through
   * @param brake Any brake object
   * @return The joystickTransmuter with the new brake layer
   */
  public JoystickTransmuter withBrake(Brake brake)
  {
    this.brake = brake;
    return this;
  }

  /**
   * Sets the list of field objects for the joystick to be processed through
   * @param fieldObjectList Any list of objects
   * @return The joystickTransmuter with the new list of objects
   */
  public JoystickTransmuter withFieldObjects(ObjectList fieldObjectList)
  {
    this.fieldObjectList = fieldObjectList;
    return this;
  }

  /**
   * Sets the inversion of the X input
   * @param invert Should the input be inverted? Default true
   * @return The JoystickTransmuter with the new inversion value
   */
  public JoystickTransmuter invertX(boolean invert)
  {
    invertX = invert;
    return this;
  }

  /**
   * Sets the inversion of the X input
   * @param invert Should the input be inverted? Default true
   * @return The JoystickTransmuter with the new inversion value
   */
  public JoystickTransmuter invertX()
  {
    invertX = true;
    return this;
  }

  /**
   * Sets the inversion of the Y input
   * @param invert Should the input be inverted? Default true
   * @return The JoystickTransmuter with the new inversion value
   */
  public JoystickTransmuter invertY(boolean invert)
  {
    invertY = invert;
    return this;
  }

  /**
   * Sets the inversion of the Y input
   * @param invert Should the input be inverted? Default true
   * @return The JoystickTransmuter with the new inversion value
   */
  public JoystickTransmuter invertY()
  {
    invertY = true;
    return this;
  }

  /**
   * Sets the rotation of the processing, used for alliance rotation
   * @param rotate Should the input be rotated?
   * @return The JoystickTransmuter with the new inversion value
   */
  public JoystickTransmuter rotated(boolean rotate)
  {
    rotateThroughput = rotate;
    return this;
  }
}
