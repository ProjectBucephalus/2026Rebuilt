package frc.robot.controlTransmutation;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** 
 * Arbitrary rotation for the input, intended for display not comp
 * @author 5985
 */
public class Rotation implements InputTransmuter
{
    private Rotation2d rotation = null;
    private Supplier<Double> angleSup = null;

    /**
     * Creates a default rotation object
     */
    public Rotation() {}

    /**
     * Creates a rotation modifier with a constant rotation value
     * @param rotationAngle Degrees anticlockwise to rotate the input, to account for clockwise rotation of the driver
     */
    public Rotation(double rotationAngle)
    {
        rotation = Rotation2d.fromDegrees(rotationAngle);
    }
    
    /**
     * Creates a rotation modifier with a supplied rotation value
     * @param angleSup Degrees anticlockwise to rotate the input, to account for clockwise rotation of the driver
     */
    public Rotation(Supplier<Double> angleSup)
    {
        this.angleSup = angleSup;
        rotation = Rotation2d.fromDegrees(angleSup.get());
    }

    public Rotation withAngleSup(Supplier<Double> angleSup)
    {
        this.angleSup = angleSup;
        rotation = Rotation2d.fromDegrees(angleSup.get());
        return this;
    }

    public Rotation withAngle(double angle)
    {
        this.angleSup = null;
        this.rotation = Rotation2d.fromDegrees(angle);
        return this;
    }

    @Override
    public Translation2d process(Translation2d controlInput)
    {
        if (rotation == null) return controlInput;

        if (angleSup != null) rotation = Rotation2d.fromDegrees(angleSup.get());
        
        return controlInput.rotateBy(rotation);
    } 
}
