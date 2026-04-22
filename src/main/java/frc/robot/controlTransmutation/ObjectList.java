package frc.robot.controlTransmutation;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/** 
 * Utility object for handling multiple FieldObjects simultaneously 
 * @author 5985
 */
public class ObjectList extends FieldObject
{
  /** 
   * List of field objects to iterate over, can include other object lists 
   * First item is highest priority, last is lowest, 
   * so that it's harder to accidentally overwrite which item is highest priority
   */
  private ArrayList<FieldObject> fieldObjects = new ArrayList<>();

  /**
   * Creates an ObjectList with any number of other field objects to process
   * @param newObjects Any number of other field objects (including 0)
   */
  public ObjectList(FieldObject ...newObjects) 
    {add(newObjects);}

  /**
   * Adds the given objects to the end of the list
   * @param newObjects list of FieldObjects to be added
   * @return this object list with the new item
   */
  public ObjectList add(FieldObject ...newObjects)
  {
    fieldObjects.addAll(List.of(newObjects));
    return this;
  }

  /**
   * Adds the given object to the start of the list so it won't be subject to edge-case conflicts </p>
   * Primarily intended for the outer wall
   * @param newObject any new FieldObject to be added as a high-priority
   * @return this object list with the new item
   */
  public ObjectList addPriority(FieldObject newObject)
  {
    fieldObjects.add(0, newObject);
    return this;
  }

  /**
   * Updates the global robot radius and position, then applies each contained object's processing to the input
   */
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    if (!globalActiveSupplier.getAsBoolean() || !activeSupplier.getAsBoolean() || fieldObjects.isEmpty()) 
      return controlInput;

    //fetchRobotValues();

    var controlOutput = controlInput;

    for (int i = fieldObjects.size() - 1; i >= 0; i--)
      controlOutput = fieldObjects.get(i).process(controlOutput);

    return controlOutput;
  }
}