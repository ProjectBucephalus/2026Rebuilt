package frc.robot.util;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link CommandXboxController} which is only active after a given button is pressed. </p>
 * All fields will return {@code 0} or {@code false} while controller is locked </p>
 * Holding the unlock button for {@value #lockTime} seconds will re-lock the controller
 * 
 * @see CommandXboxController
 */
public class LockableXboxController extends CommandXboxController
{
  private static final double lockTime = 1.5;
  private static final double rumbleTime = 0.2;

  private final XboxController io_Hid;

  private boolean unlocked = false;
  private Trigger isUnlocked = new Trigger(() -> unlocked);

  /**
   * Construct an instance of a controller, starts locked.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   * @param lockButton Pressing this button unlocks the controller, holding for 2 seconds re-locks it.
   */
  public LockableXboxController(int port, Button lockButton) 
  {
    super(port);
    io_Hid = new XboxController(port);
    new Trigger(() -> io_Hid.getRawButtonPressed(lockButton.value) && !unlocked)
      .onTrue
      (
        run(() -> io_Hid.setRumble(RumbleType.kBothRumble, 1.0))
          .withTimeout(rumbleTime)
          .andThen(runOnce
          (() -> {
            unlocked = true;
            io_Hid.setRumble(RumbleType.kBothRumble, 0.0);
          }))
        .ignoringDisable(true)
      );

    new Trigger(() -> io_Hid.getRawButton(lockButton.value))
      .whileTrue(waitSeconds(lockTime - rumbleTime)
      .andThen(run(() -> io_Hid.setRumble(RumbleType.kBothRumble, 1.0))
        .withTimeout(rumbleTime))
      .andThen(runOnce(
        () -> {
          unlocked = false;
          io_Hid.setRumble(RumbleType.kBothRumble, 0.0);
        }))
      .ignoringDisable(true));
  }

  @Override public Trigger a() {return isUnlocked.and(super.a());}
  @Override public Trigger b() {return isUnlocked.and(super.b());}
  @Override public Trigger x() {return isUnlocked.and(super.x());}
  @Override public Trigger y() {return isUnlocked.and(super.y());}
  @Override public Trigger leftBumper() {return isUnlocked.and(super.leftBumper());}
  @Override public Trigger rightBumper() {return isUnlocked.and(super.rightBumper());}
  @Override public Trigger back() {return isUnlocked.and(super.back());}
  @Override public Trigger start() {return isUnlocked.and(super.start());}
  @Override public Trigger leftStick() {return isUnlocked.and(super.leftStick());}
  @Override public Trigger rightStick() {return isUnlocked.and(super.rightStick());}
  
  @Override public Trigger leftTrigger() {return isUnlocked.and(super.leftTrigger());}
  @Override public Trigger leftTrigger(double threshold) {return isUnlocked.and(super.leftTrigger(threshold));}
  @Override public Trigger rightTrigger() {return isUnlocked.and(super.rightTrigger());}
  @Override public Trigger rightTrigger(double threshold) {return isUnlocked.and(super.rightTrigger(threshold));}

  @Override public double getLeftX() {return unlocked ? super.getLeftX() : 0;}
  @Override public double getLeftY() {return unlocked ? super.getLeftY() : 0;}
  @Override public double getRightX() {return unlocked ? super.getRightX() : 0;}
  @Override public double getRightY() {return unlocked ? super.getRightY() : 0;}
  @Override public double getLeftTriggerAxis() {return unlocked ? super.getLeftTriggerAxis() : 0;}
  @Override public double getRightTriggerAxis() {return unlocked ? super.getRightTriggerAxis() : 0;}

}
