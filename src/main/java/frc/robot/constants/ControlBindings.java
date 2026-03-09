package frc.robot.constants;

/** Mappings for control bindings, planned to explore the option of containing functions to bind controls */
public class ControlBindings 
{
  /* 

  // -------------STATE--------------- //

  [auto-aim / auto-pass / auto-shoot]
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

  [nudging]
  if (nudging && in bump zone)
    {nudge to nearest 30 degrees away from cardinal}
  
  if (nudging && in trench zone)
    {nudge to nearest 180 degrees}

  // -------------DRIVER-------------- //
  
  *driver.rightX -> rotation
  *driver.leftXY -> translation
  *driver.rightTriggerAxis -> brakes

  *driver.leftTrigger.onTrue -> deploy intake
  *driver.leftTrigger.whileTrue -> run intake // Run while deployed, should start after deploying
  
  *driver.leftBumper -> manual shoot -> ensure flywheels at least idle speed, then run indexers
  *driver.rightBumper -> stop indexers and shooters idle, overides everything

  driver.b -> ?? bump rotation lock ??
  driver.y -> trench rotation lock -> rotate on press, heading straight towards other zone
  driver.x -> tower rotation lock -> based on selected clime location, enable attractor
  driver.a -> outpost rotation lock -> face in or right, whichever is closer on press

  *driver.povUp.whileTrue -> agitate and run intake
  *driver.povUp.onFalse -> deploy intake
  *driver.povDown -> retract intake

  *driver.back -> disable bump and trench nudging
  *driver.start -> enable bump and trench nudging // bump ~30 degrees from cardinal, trench 0 or 180

  // -------------DEBUG--------------- //

  *debug.y.onTrue -> unlock controller
  *debug.y.whileTrue -> if held for 2 seconds, lock controller

  *debug.x.whileTrue -> agitate and run intake
  *debug.x.onFalse -> deploy intake

  *debug.a -> run intake
  *debug.b -> reverse intake // should win

  *debug.povUp/Down -> intake manual position, should ignore calibration
  
  ~debug.leftTrigger -> run port flywheel and indexer, return to previous state on release // ?? what speed ??
  ~debug.leftBumper -> port shooter idle, reverse indexer, return to previous state on release
  ~debug.rightTrigger -> run stbd flywheel and indexer, return to previous state on release // ?? what speed ??
  ~debug.rightBumper -> stbd shooter idle, reverse indexer, return to previous state on release 

  *debug.rightStick -> manual shooter aim mode
  *debug.rightXY -> shooter aim control

  *debug.back -> climber retract
  *debug.start -> climber deploy
  *debug.leftY -> manual climber control

  // -------------CASE---------------- //

  // All switch bindings should be `onTrue` and `onFalse`, to allow other systems to overide them

  ~enable/disable fencing switch
  ~enable/disable vision switch

  ~auto-aim-switch
  ~auto-pass switch
  ~auto-shoot switch

  ~activate climb, (double buttons must both be pressed?)

  // -------------BTN-PAD------------- //

     A B C D E F G H  M
  1 [][][][][][][][] ()
  2 [][][][][][][][] ()
  3 [][][][][][][][] ()
  4 [][][][][][][][] ()
  5 [][][][][][][][] ()
  6 [][][][][][][][] ()
  7 [][][][][][][][] ()
  8 [][][][][][][][] ()

  A4-H8 -> Alliance Zone map
  M1 -> Full auto targeting, reset target -> map sets pass point
  M2 -> Position Mode -> map sets robot position
  M4 -> Manual controlls
  M6 -> Disable e-stop
  M8 -> Enable e-stop

  Port Shooter:
  Normal set:
  A1 - Disable
  A2 - Drop
  B2 - Idle
  C1 - Shoot
  C2 - Unjam

  Manual set:
  A1 - Rev
  A2 - Idle
  A3 - Stop
  A4 - Reverse
  B2 - Left
  D2 - Right
  C1 - Up
  C2 - Down
  B4 - Feed
  C4 - Stop feeder
  D4 - Reverse feeder

  Stbd Shooter:
  Normal set:
  H1 - Disable
  H2 - Drop
  G2 - Idle
  F1 - Shoot
  F2 - Unjam

  Manual set:
  H1 - Rev
  H2 - Idle
  H3 - Stop
  H4 - Reverse
  E2 - Left
  G2 - Right
  F1 - Up
  F2 - Down
  G4 - Feed
  F4 - Stop feeder
  E4 - Reverse feeder

  Intake:
  Normal set:
  D1 - Run
  D2 - Agitate
  D3 - Deploy
  E1 - Reverse
  E2 - Squish
  E3 - Stow

  Manual set:
  A6 - Run
  A7 - Reverse
  B6 - Out
  C6 - In
  B7 - Deply
  C7 - Stow
  B8 - Agitate
  C8 - Squish

  Climber:
  Normal set:
  B3 - Left Climb
  G3 - Right Climb

  Manual set:
  F6 - Out
  G6 - In
  F7 - Deploy
  G7 - Stow
  F8 - Left Climb
  G8 - Right Climb

  */
}
