package frc.robot.util.controlTransmutation;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;

/** Utility object for handling multiple FieldObjects simultaneously */
public class ObjectList extends FieldObject
{
  /** List of field objects to itterate over, can include other object lists */
  private ArrayList<FieldObject> fieldObjects;

  /**
   * Creates an ObjectList with any number of other field objects to process
   * @param newObjects Optional list of other field objects
   */
  public ObjectList()
  {
    fieldObjects = new ArrayList<FieldObject>();
  }

  /**
   * Creates an ObjectList with any number of other field objects to process
   * @param newObjects Optional list of other field objects
   */
  public ObjectList(FieldObject ...newObjects) 
  {
    this();
    add(newObjects);
  }

  public Translation2d process(Translation2d controlInput)
  {
    if (!activeSupplier.getAsBoolean() || fieldObjects.size() <= 0) 
      return controlInput;

    fetchRobotPos();

    var controlOutput = controlInput;
    for (int i = fieldObjects.size() - 1; i >= 0; i--)
    {
      controlOutput = fieldObjects.get(i).process(controlOutput);
    }
    return controlOutput;
  }

  /**
   * Adds the given object to the end of the list
   * @param newObjects list of FieldObjects to be added
   * @return this object list with the new item
   */
  public ObjectList add(FieldObject ...newObjects)
  {
    for (FieldObject object : newObjects) 
    {
      fieldObjects.add(object);
    }
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
}