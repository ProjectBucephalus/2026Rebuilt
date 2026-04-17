package frc.robot.subsystems;

import java.util.HashSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * A subsystem that provides a convenient interface for managing controller rumble. Each instance manages one side of one controller <p>
 * This subsystem operates on requests. It rumbles while there are any requests, and when all requests are removed it stops rumbling
 * @author 5985
 */
public class RumbleRequester extends SubsystemBase
{
  private HashSet<String> queue = new HashSet<>();
  private final CommandXboxController controller;
  private final RumbleType side;
  private final Supplier<Double> strengthSup;

  /**
   * Creates a RumbleRequester, for managing a controller's rumble 
   * @param controller the controller to manage the rumble of
   * @param side which side of the controller this subsystem maanages
   * @param strengthSup the supplier that is used to get the rumble strength each time it starts rumbling
   */
  public RumbleRequester(CommandXboxController controller, RumbleType side, Supplier<Double> strengthSup)
  {
    this.controller = controller;
    this.side = side;
    this.strengthSup = strengthSup;
  }

  /**
   * Add a new rumble request <p>
   * If the ID already exists in the queue, this will do nothing
   * @param rumbleID the ID of the new request, primarily used to remove it
   */
  private void add(String rumbleID)
    {queue.add(rumbleID);}

  /**
   * Remove an existing rumble request <p>
   * If the ID doesn't exist in the queue, this will do nothing
   * @param rumbleID the ID of the request to remove
   */
  private void remove(String rumbleID)
    {queue.remove(rumbleID);}

  /**
   * Clears all current requests from the queue, immediately ending the rumble
   */
  public void clear()
    {queue.clear();}

  /**
   * Construct a command that adds a request when it's scheduled, and removes the request when it ends
   * 
   * @param rumbleID the ID to use for the request
   * @return the {@link Command}
   */
  public Command runRumbleCmd(String rumbleID)
    {return Commands.startEnd(() -> add(rumbleID), () -> remove(rumbleID));}

  /**
   * Bind the provided trigger to run a rumble request while it is true
   * 
   * @param rumbleID the ID to use for the request
   * @param trigger the trigger to bind the request to
   * @return this subsystem, for easier chaining
   */
  public RumbleRequester addRumbleTrigger(String rumbleID, Trigger trigger)
  {
    trigger.whileTrue(runRumbleCmd(rumbleID));
    return this;
  }

  /**
   * Construct a command that runs a rumble request for the provided duration <p>
   * The rumble will also stop if the command is interrupted
   * 
   * @param rumbleID the ID to use for the request
   * @param durationSeconds how long to rumble for in seconds, measured from when the command starts
   * @return the {@link Command}
   */
  public Command timedRumbleCmd(String rumbleID, double durationSeconds)
    {return runRumbleCmd(rumbleID).withDeadline(Commands.waitSeconds(durationSeconds));}

  @Override
  public void periodic() 
    {controller.setRumble(side, queue.isEmpty() ? 0 : strengthSup.get());}
}
