package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.robot.constants.FieldConstants.*;

import java.util.Optional;
import frc.robot.Robot;

import frc.robot.constants.Constants.ShooterConstants;

import static frc.robot.constants.Constants.SwerveConstants.robotRadiusInscribed;

/** 
 * Field or FMS related utilities 
 * @author 5985
 */
public final class FieldUtils 
{
  private static Alliance alliance;
  static {updateAlliance();}

  private static Optional<Alliance> autoWinner = Optional.empty();

  public static Optional<Alliance> getAutoWinner()
    {return autoWinner;}

  /** 
   * Attempts to fetch the alliance that won auto from DriverStation, if we haven't already got it </p>
   * Randomly assigns a winner when running Auto in simulation
   */
  public static void updateAutoWinner()
  {
    if (autoWinner.isEmpty()) 
    {
      String gameData = DriverStation.getGameSpecificMessage();
      if (!gameData.isEmpty())
        autoWinner = switch (gameData.charAt(0))
        {
          case 'B' -> Optional.of(Alliance.Blue);
          case 'R' -> Optional.of(Alliance.Red);
          default  -> Optional.empty();
        };
        
      else if (Robot.isSimulation() && DriverStation.isAutonomous())
        autoWinner = Math.rint(Math.random()) == 0 ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
    }
  }

  /** @return whether the provided alliance's hub is active, with margin on each side to maximise scoring */
  public static boolean hubActiveToleranced(Alliance alliance, double preMargin, double postMargin) 
  {
    double timeElapsed = MatchTime.getTeleTimeElapsed();

    if (hubBothToleranced(preMargin, postMargin))
      return true;
    else
    {
      if (alliance == autoWinner.get()) 
        return (timeElapsed >= (35 - preMargin) && timeElapsed < (60 + postMargin)) // Shift 2
        || (timeElapsed >= (85 - preMargin) && timeElapsed < (110 + postMargin));   // Shift 4
      else 
        return (timeElapsed >= (10 - preMargin) && timeElapsed < (35 + postMargin)) // Shift 1
        || (timeElapsed >= (60 - preMargin) && timeElapsed < (85 + postMargin));    // Shift 3
    }     
  }

  /** @return whether both hubs are active together, with margin on each side */
  public static boolean hubBothToleranced(double preMargin, double postMargin)
  {
    double timeElapsed = MatchTime.getTeleTimeElapsed();

    return
    (
      autoWinner.isEmpty()                 // Don't know yet
      || timeElapsed == 0                  // Auto
      || timeElapsed < (10 + postMargin)   // Transition
      || timeElapsed >= (110 - preMargin)  // Endgame
    );
  }

  /** @return whether the our alliance's hub is active, with margin on each side to maximise scoring */
  public static boolean hubActiveToleranced(double preMargin, double postMargin)
    {return hubActiveToleranced(getAlliance(), preMargin, postMargin);}

  /** @return whether the provided alliance's hub is active */
  public static boolean hubActive(Alliance alliance) 
    {return hubActiveToleranced(alliance, 0, 0);}

  /** @return whether our alliance's hub is active */
  public static boolean hubActive() 
    {return hubActiveToleranced(0, 0);}

  /** @return whether the provided alliance's hub will become active within the given margin */
  public static boolean hubTransition(Alliance alliance, double preMargin)
  {
    double timeElapsed = MatchTime.getTeleTimeElapsed();

    if (autoWinner.isEmpty()) return false;
    
    if (alliance == autoWinner.get()) 
      return (timeElapsed >= (35 - preMargin) && timeElapsed < (35)) // Shift 2
      || (timeElapsed >= (85 - preMargin) && timeElapsed < (85));    // Shift 4
    else 
      return (timeElapsed >= (10 - preMargin) && timeElapsed < (10)) // Shift 1
      || (timeElapsed >= (60 - preMargin) && timeElapsed < (60));    // Shift 3
  }
  
  /** @return whether both hubs will become active within the given margin */
  public static boolean hubBothTransition(double preMargin)
    {return !hubBothToleranced(0, 0) && hubBothToleranced(preMargin, 0);}

  /**
   * Checks whether we are on the red alliance <p>
   * If the alliance value is unavailable for some reason, it will always return false (i.e. blue alliance)
   * 
   * @return true if we are on the red alliance
   */
  public static boolean isAlliance(Alliance testAlliance) 
    {return alliance == testAlliance;}

  public static Alliance getAlliance() 
    {return alliance;}

  /**
   * Updates the cached alliance value 
   */
  public static void updateAlliance()
    {alliance = DriverStation.getAlliance().orElse(Alliance.Blue);}

  /** 
   * @return the centre point of your alliance's hub
   */
  public static Translation2d getAllianceHubCentre() 
    {return getHubCentre(getAlliance());}

  /** 
   * @return the centre point the alliance's hub
   */
  public static Translation2d getHubCentre(Alliance alliance) 
  {
    return switch (alliance) 
    {
      case Blue -> blueHubCentre;
      case Red -> redHubCentre;
    };
  }

  /**
   * @return which driver station we are being controlled from (1, 2, or 3), or 0 if the value is unavailable
   */
  public static final int getDriverLocation()
    {return DriverStation.getLocation().orElse(0);}

  /**
   * Mirrors the provided pose if we're on the red alliance
   * 
   * @param pose a blue-origin pose
   * @return the pose mirrored to match our alliance
   */
  public static Pose2d allianceFlipPose(Pose2d pose) 
  {
    // flip pose when red
    if (isAlliance(Alliance.Red)) 
      return flipPose(pose);
    else
      // Blue or we don't know; return the original pose
      return pose;
  }

  /**
   * Mirrors the provided pose
   * 
   * @param pose original pose
   * @return the pose mirrored accross field centreline
   */
  public static Pose2d flipPose(Pose2d pose) 
  {
    Rotation2d rot = pose.getRotation();
    // reflect the pose over center line, flip both the X and the rotation
    return new Pose2d(fieldLength - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
  }

  /**
   * Rotates the provided pose if we're on the red alliance
   * 
   * @param pose a blue-origin pose
   * @return the pose rotated to match our alliance
   */
  public static Pose2d allianceRotatePose(Pose2d pose) 
  {
    // flip pose when red
    if (isAlliance(Alliance.Red)) 
      // reflect the pose around center point, flip both the X and Y position and rotation
      return pose.rotateAround(fieldCentre, Rotation2d.k180deg);
    else 
      // Blue or we don't know; return the original pose
      return pose;
  }

  /**
   * Rotates the provided rotation if we're on the red alliance
   * 
   * @param pose a blue-origin rotation
   * @return the rotation rotated to match our alliance
   */
  public static Rotation2d allianceRotateRotation(Rotation2d rotation) 
  {
    return isAlliance(Alliance.Red) ? rotation.rotateBy(Rotation2d.k180deg) : rotation;
  }

  /**
   * Rotates the provided pose
   * 
   * @param pose original pose
   * @return the pose rotated around the field centre
   */
  public static Pose2d rotatePose(Pose2d pose) 
  {
    // reflect the pose around center point, flip both the X and Y position and rotation
    return pose.rotateAround(fieldCentre, Rotation2d.k180deg);
  }

  /**
   * Mirrors the provided translation accross the field centreline if we're on the red alliance
   * 
   * @param translation a blue-origin translation
   * @return the translation mirrored to match our alliance
   */
  public static Translation2d allianceFlipTranslation(Translation2d translation) 
  {
    // flip translation when red
    if (isAlliance(Alliance.Red)) 
      // reflect the translation around center point, flip both the X and Y position
      return flipTranslation(translation);
    else 
      // Blue or we don't know; return the original translation
      return translation;
  }

  /**
   * Mirrors the provided translation accross the field centreline
   * 
   * @param translation original translation
   * @return the translation mirrored accross field centre
   */
  public static Translation2d flipTranslation(Translation2d translation) 
  {
    // reflect the translation around center point, flip both the X and Y position
    return translation.rotateAround(fieldCentre, Rotation2d.k180deg);
  }

  /**
   * Rotates the provided translation if we're on the red alliance
   * 
   * @param translation a blue-origin translation
   * @return the translation rotated to match our alliance
   */
  public static Translation2d allianceRotateTranslation(Translation2d translation) 
  {
    // flip translation when red
    if (isAlliance(Alliance.Red)) 
      // reflect the translation around center point, flip both the X and Y position
      return rotateTranslation(translation);
    else 
      // Blue or we don't know; return the original translation
      return translation;
  }

  /**
   * Rotates the provided translation
   * 
   * @param translation original translation
   * @return the translation rotated around field centre
   */
  public static Translation2d rotateTranslation(Translation2d translation) 
  {
    // reflect the translation around center point, flip both the X and Y position
    return translation.rotateAround(fieldCentre, Rotation2d.k180deg);
  }

  /**
   * Checks if the provided position is within our alliance zone
   * 
   * @param pos Position to check against
   * @return If the position is within the alliance zone
   */
  public static boolean inAllianceZone(Translation2d pos) 
  {
    return switch (getAlliance())
    {
      case Blue -> pos.getX() < blueStartLine.getX() + robotRadiusInscribed;
      case Red -> pos.getX() > redStartLine.getX() - robotRadiusInscribed;
    };
  }

  public static Translation2d getPassPoint(Translation2d pos)
  {
    boolean inLeftHalf = switch (getAlliance())
    {
      case Blue -> pos.getY() > fieldCentre.getY();
      case Red -> pos.getY() < fieldCentre.getY();
    };

    var point = ShooterConstants.passPoint.get();
    
    if (inLeftHalf) point = new Translation2d(point.getX(), fieldWidth - point.getY());

    double maxRangeSqrd = ShooterConstants.maxPassRange * ShooterConstants.maxPassRange;
    // Behaviour is the same as `pos.getDistance(point) > maxPassRange`, 
    // but avoids computationally expensive square root function
    if (pos.getSquaredDistance(point) > maxRangeSqrd)
    {
      double dy = point.getY() - pos.getY();
      double dx = Math.sqrt(maxRangeSqrd - (dy * dy));
      //        pos
      // =--dy--/
      // |     /
      // |   maxRange
      // dx  /
      // |  /
      // | /
      // =/
      // |
      // |
      // point

      double newX = switch (getAlliance())
      {
        case Blue -> pos.getX() - dx;
        case Red -> pos.getX() + dx;
      };
      point = new Translation2d(newX, point.getY());
    }

    return point;
  }  
}