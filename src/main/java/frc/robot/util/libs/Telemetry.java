package frc.robot.util.libs;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

/**
 * Mechanism data-logging system
 * @author CTRE
 */
public class Telemetry 
{
  /**
   * Construct a telemetry object, with the specified max speed of the robot
   * 
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry() {SignalLogger.start();}

  private final double[] m_poseArray = new double[3];
  private final double[] m_moduleStatesArray = new double[8];
  private final double[] m_moduleTargetsArray = new double[8];

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    /* Also write to log file */
    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    for (int i = 0; i < 4; ++i) 
    {
      m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
      m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
      m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
      m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
    }

    SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
  }
}
