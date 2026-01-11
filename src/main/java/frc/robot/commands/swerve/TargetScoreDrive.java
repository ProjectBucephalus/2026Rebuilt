package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import static frc.robot.constants.FieldConstants.GeoFencing.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

public class TargetScoreDrive extends HeadingLockedDrive 
{
  private Rotation2d rotationOffsetBase;

  /** Creates a new TargetScoreDrive. */
  public TargetScoreDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Translation2d> joystickSupplier,
    Rotation2d rotationOffset,
    Supplier<Translation2d> robotPosSup
  ) 
  {
    super(s_Swerve, joystickSupplier, Rotation2d.kZero, rotationOffset, robotPosSup);
    this.rotationOffsetBase = rotationOffset;
  }

  @Override
  protected void updateTargetHeading()
  { 
    if (reefBlue.getDistance() >= robotRadiusCircumscribed/2 && reefRed.getDistance() >= robotRadiusCircumscribed/2)
    {
      switch (FieldUtils.getNearestReefFace(robotXY)) 
      {
        case 1 ->
        {
          targetHeading = Rotation2d.kZero;
          super.rotationOffset = this.rotationOffsetBase;
        }

        case 2 ->
        {
          targetHeading = new Rotation2d(Units.degreesToRadians(60));
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
        }

        case 3 ->
        {
          targetHeading = new Rotation2d(Units.degreesToRadians(120));
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
        }

        case 4 ->
        {
          targetHeading = Rotation2d.k180deg;
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
        } 

        case 5 ->
        {
          targetHeading = new Rotation2d(Units.degreesToRadians(-120));
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
        }  
        
        case 6 ->
        {
          targetHeading = new Rotation2d(Units.degreesToRadians(-60));
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
        }
      }
    }
  }
}