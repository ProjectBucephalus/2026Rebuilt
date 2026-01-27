package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import frc.robot.constants.Constants.Interpolation;
import static frc.robot.constants.Constants.Shooter.HoodConstants.*;

public class Hood {
  private final Servo m_Servo;
  private final Transform2d shooterOffset;

  public Hood(int id, Transform2d shooterOffset)
  {
    m_Servo = new Servo(id);
    this.shooterOffset = shooterOffset;
  }

  // calculates the distance to the target from the shooter
  private double calculateTargetDist(Pose2d robotPose, Translation2d targetPoint)
  {
    //get the current rbot position and adds the offset of the shooter to this in order to get the position of the the shooter 
    var shooterPose = robotPose.plus(shooterOffset);
    double targetDist = targetPoint.minus(shooterPose.getTranslation()).getNorm();
   
    return targetDist;
  }

  public Rotation2d getAltitude()
  {
    return Rotation2d.fromDegrees(m_Servo.getAngle());
  }

  public void update(Pose2d robotPose, Target target)
  {
    var targetAltitude = switch (target.state) 
    {
      // fixed angle 
      case Manual -> target.altitude;
      //aimed at a point on the field that can be changed 
      case Point -> Rotation2d.fromDegrees(Interpolation.shooterAltitudeLow.get(calculateTargetDist(robotPose, target.point)));
      //always aimed at hub
      case Hub -> Rotation2d.fromDegrees(Interpolation.shooterAltitudeHub.get(calculateTargetDist(robotPose, FieldUtils.getAllianceHubCentre())));
    };

    target.altitude = targetAltitude;

    // sets the angle of m_Servo to targetAngle while only being able to go to minAngle or maxAngle
    m_Servo.set(Conversions.clamp(targetAltitude.getDegrees(), 0, hoodRange) / (servoRange * hoodRatio));
  }
}
