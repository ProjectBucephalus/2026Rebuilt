package frc.robot.util.autobuilder;

import frc.robot.util.PBDash;

public class AutoBuilder 
{
  private static boolean hadError = false;

  protected static void error(String message) 
  {
    hadError = true;
    PBDash.print(PBDash.AUTO_ERRS, message, ", ");
  }  
}
