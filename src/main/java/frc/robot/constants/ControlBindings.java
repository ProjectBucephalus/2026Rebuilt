package frc.robot.constants;

/** Mappings for control bindings, planned to explore the option of containing functions to bind controls */
public class ControlBindings 
{
  /* 

  // -------------STATE--------------- //

  auto-aim / auto-pass / auto-shoot
  if (auto-aim)
  {
    automatically aim at hub when in alliance zone

    if (auto-pass)
    {
      aim at closest pass point
    }
    else
    {
      aim at operator selected pass point
    }
    
    if (auto-shoot)
    {
      rev flywheels and shoot when ready
    }
    else
    {
      flywheels idle until told to shoot
    }
  }
  else
  {
    flywheels idle, manual aim
    when told to shoot, rev based on manual distance and shoot when ready
  }

  // -------------DRIVER-------------- //
  
  driver.rightX -> rotation
  driver.leftXY -> translation
  driver.rightTriggerAxis -> brakes

  driver.leftTrigger.onTrue -> deploy intake
  driver.leftTrigger.whileTrue -> run intake // Run while deployed, should start after deploying
  
  driver.leftBumper -> manual shoot -> ensure flywheels at least idle speed, then run indexers
  driver.rightBumper -> stop indexers and shooters idle, overides everything

  driver.b -> ?? bump rotation lock ??
  driver.y -> trench rotation lock -> rotate on press, heading straight towards other zone
  driver.x -> tower rotation lock -> based on selected clime location, enable attractor
  driver.a -> outpost rotation lock -> face in or right, whichever is closer on press

  driver.povUp.whileTrue -> agitate and run intake
  driver.povUp.onFalse -> deploy intake
  driver.povDown -> retract intake

  driver.back -> disable bump and trench nudging
  driver.start -> enable bump and trench nudging // bump ~30 degrees from cardinal, trench 0 or 180

  // -------------DEBUG--------------- //

  debug.y.onTrue -> unlock controller
  debug.y.whileTrue -> if held for 2 seconds, lock controller

  debug.x.whileTrue -> agitate and run intake
  debug.x.onFalse -> deploy intake

  debug.a -> run intake
  debug.b -> reverse intake

  debug.povUp/Down -> intake manual position, should ignore calibration
  
  debug.leftTrigger -> run ?left/port? flywheel and indexer, return to previous state on release // ?? what speed ??
  debug.leftBumper -> ?left/port? shooter idle, reverse indexer, return to previous state on release
  debug.rightTrigger -> run ?right/stbd? flywheel and indexer, return to previous state on release // ?? what speed ??
  debug.rightBumper -> ?lright/stbd? shooter idle, reverse indexer, return to previous state on release 

  debug.rightStick -> manual shooter aim mode
  debug.rightXY -> shooter aim control

  debug.back -> climber retract
  debug.start -> climber deploy
  debug.leftY -> manual climber control

  // --------------------------------- //



  */
}
