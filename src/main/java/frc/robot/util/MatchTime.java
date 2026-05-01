package frc.robot.util;

/** Utilities for tracking match time. Make sure {@link MatchTime#startAuto startAuto()} and {@link MatchTime#startTele startTele()} are called at the appropriate times */
public class MatchTime 
{
  private static final double AUTO_TIME = 20;        //Auto length in millis
  private static final double TELE_TIME = 140;       //Teleop length in millis
  private static final double MAX_GAME_TIME = AUTO_TIME + TELE_TIME;   //match length in millis

  // timer function variables
  private static double autoStart = 0;            //variable to save system time at start of auto
  private static double teleStart = 0;            //variable to save system time at start of teleop

  /**
  * internal wrapper for system time to return in seconds instead of millis 
  * @return current system time in seconds.
  */
  public static double currentTime()
  {
    return (double)System.currentTimeMillis()/1000;
  }

  /**
   * Called at the start of auto to mark the beginning of the match
   *  
   */ 
  public static void startAuto()
  {
    autoStart = currentTime();
  }

  /**
   * Called at the start of teleop to mark the beginning of the teleop period
   * 
   */
  public static void startTele()
  {
    teleStart = currentTime();
  }

  /**
   * Gets game time elapsed in seconds.
   * 
   * @return the number of seconds since the startAuto() call,
   * returns zero if startAuto() has not been called or the match is over.
   */
  public static double getGameTimeElapsed()
  {
    double timeNow = currentTime();
    if ((autoStart == 0) || (timeNow > (autoStart + MAX_GAME_TIME)))
    {
      return 0;
    }
    else
    {
      return timeNow-autoStart;
    }
  }

  /**
   * Gets the number of seconds remaining in the current match.
   * 
   * @return the time remaining in the current match, in seconds.
   * Returns zero if startAuto has not been called or the match is over.
   */
  public static double getGameTimeRemaining()
  {
    double timeNow = currentTime();
    if ((autoStart == 0) || (timeNow > (autoStart + MAX_GAME_TIME)))
    {
      return 0;
    }
    else
    {
      return MAX_GAME_TIME - (timeNow - autoStart);
    }
  }

  /**
   * Gets the time elapsed in the current autonomous period.
   * NB while in auto should be identical to getGameTimeElapsed()
   * 
   * @return time elapsed since the startAuto() call, in seconds.
   * Will return zero if startAuto() has not been called, and {@value MatchTime#AUTO_TIME} if auto is finished.
   */
  public static double getAutoTimeElapsed()
  {
    double timeNow = currentTime();
    if (timeNow < autoStart)
    {
      return 0;
    }
    else
    {
      return Math.min(timeNow - autoStart, AUTO_TIME);
    }
  }

  /**
   * Gets the time remaining in the current autonomous period.
   * 
   * @return the time remaining in the current auto, in seconds.
   * Will return zero if startAuto() has not been called, or if auto is finished.
   */
  public static double getAutoTimeRemaining()
  {
    double timeNow = currentTime();
    if ((timeNow < autoStart) || (timeNow > autoStart + AUTO_TIME))
    {
      return 0;
    }
    else
    {
      return AUTO_TIME - (timeNow - autoStart);
    }
  }

  /**
   * Gets the time elapsed during the current teleoperated period.
   * 
   * @return the time elapsed since the startTele() call, in seconds.
   * Will return zero if startTele() has not been called, or the match is over.
   */
  public static double getTeleTimeElapsed()
  {
    double timeNow = currentTime();
    if ((timeNow < teleStart) || (timeNow > (teleStart + TELE_TIME)))
    {
      return 0;
    }
    else
    {
      return timeNow - teleStart;
    }
  }

  /**
   * Gets the time remaining in the current teleoperated period.
   * 
   * @return the time remaining in the current teleop, in seconds.
   * Will return zero if startTele() has not been called, or the match is over.
   */
  public static double getTeleTimeRemaining()
  {
    double timeNow = currentTime();
    if ((timeNow < teleStart) || (timeNow > (teleStart + TELE_TIME)))
    {
      return 0;
    }
    else
    {
      return TELE_TIME - (timeNow - teleStart);
    }
  }  
}
