package frc.robot.util.autobuilder;

import java.util.Arrays;

import frc.robot.util.autobuilder.ParsedRepr.Value.Type;

/** A number of types used for representing the parsed form of the auto string */
public final class ParsedRepr 
{
  private ParsedRepr() {}

  /** An exception thrown when attempting to get a {@link ParsedRepr.Value Value} as a type that it isn't */
  public static class TypeMismatchException extends RuntimeException 
  {
    /** The expected type */
    public final Type expected;
    /** The actual value found */
    public final Value found;

    public TypeMismatchException(Type expected, Value found)
    {
      this.expected = expected;
      this.found = found;
    }
  }

  /** 
   * A value with an attached type <p>
   * The value itself is stored as an Object, which is cast appropriately depending on the type.
   * It will therefore cause a runtime exception if: <ul>
   * <li> A value of type Num does not contain a {@code Double}
   * <li> A value of type Bool does not contain a {@code Boolean}
   * <li> A value of type String does not contain a {@code String}
   */
  public static record Value(Type type, Object value)
  {
    /** The possible types of a value */
    public static enum Type 
    {
      /** Number type, represented as a double */
      Num,
      /** Boolean type, which uses {@code on} and {@code off} as it's literals */
      Bool,
      /** String type, limited to {@code a..z}, {@code 0..9}, and {@code _}. Cannot start with a digit */
      String
    }

    /** @return The underlying double value, or throws a {@link ParsedRepr.TypeMismatchException TypeMismatchException} if this value is not a Num */
    public double asNum()
    {
      return switch (type)
      {
        case Num -> ((Double)value).doubleValue();
        default -> throw new TypeMismatchException(Type.Num, this);
      };
    }

    /** @return The underlying boolean value, or throws a {@link ParsedRepr.TypeMismatchException TypeMismatchException} if this value is not a Bool */
    public boolean asBool()
    {
      return switch (type)
      {
        case Bool -> value == "on" ? true : false; 
        default -> throw new TypeMismatchException(Type.Bool, this);
      };
    }

    /** @return The underlying String value, or throws a {@link ParsedRepr.TypeMismatchException TypeMismatchException} if this value is not a String */
    public String asString()
    {
      return switch (type)
      {
        case String -> (String)value;
        default -> throw new TypeMismatchException(Type.String, this);
      };
    }

    @Override
    public final String toString() 
    {
      return value.toString() + ": " + type.toString();
    }
  }

  /** An instruction, formed of a instruction type and an array of {@link ParsedRepr.Value Values} */
  public static record Instruction(Type type, Value... args) 
  {
    /** The possible types of instruction. 
     * Must all be lowercase, as the parser converts a lowercase String to this enum via the built-in {@link Enum#valueOf valueOf} method 
     */
    public static enum Type
    {
      driveto, follow, 
      waitfor, waituntil, 
      intake, 
      passing
    }

    /** @return The argument at index {@code i} */
    public Value arg(int i)
      {return args[i];}

    @Override
    public final String toString() 
    {
      return type.toString() + Arrays.toString(args);
    }
  }
}
