package frc.robot.subsystems;

import java.util.HashSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RumbleRequester extends SubsystemBase
{
  private HashSet<String> queue = new HashSet<String>();
  private final CommandXboxController controller;
  private final RumbleType side;
  private final Supplier<Double> strengthSup;

  public RumbleRequester(CommandXboxController controller, RumbleType side, Supplier<Double> strengthSup)
  {
    this.controller = controller;
    this.side = side;
    this.strengthSup = strengthSup;
  }

  private void add(String rumbleID)
    {queue.add(rumbleID);}

  private void remove(String rumbleID)
    {queue.remove(rumbleID);}

  public void clear()
    {queue.clear();}

  /** Returns a command that runs a rumble while it's running */
  public Command rumbleWhileCommand(String rumbleID)
    {return Commands.startEnd(() -> add(rumbleID), () -> remove(rumbleID));}

  /** Adds a rumble with the provided ID that runs while the given trigger is true. Returns the subsystem for easier chaining. */
  public RumbleRequester addRumbleTrigger(String rumbleID, Trigger trigger)
  {
    trigger.whileTrue(Commands.startEnd(() -> add(rumbleID), () -> remove(rumbleID)));
    return this;
  }

  /** Returns a command that runs a rumble for the provided duration */
  public Command timedRequestCommand(String rumbleID, double durationSeconds)
    {return rumbleWhileCommand(rumbleID).withDeadline(Commands.waitSeconds(durationSeconds));}

  @Override
  public void periodic() 
  {
    controller.setRumble(side, queue.isEmpty() ? 0 : strengthSup.get());
  }
}
