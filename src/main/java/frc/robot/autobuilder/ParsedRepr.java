package frc.robot.autobuilder;

import java.io.Serializable;
import java.util.Arrays;

import frc.robot.autobuilder.ParsedRepr.Value.Type;

/** A number of types used for representing the parsed form of the auto string */
public final class ParsedRepr 
{
  private ParsedRepr() {}

  /** An exception thrown when an {@link ParsedRepr.Instruction Instruction} has the incorrect number of arguments */
  public static class ArgCountException extends Exception 
  {
    /** The expected number of arguments */
    public final int expected;
    /** The actual number found found */
    public final int found;

    public ArgCountException(int expected, int found)
    {
      this.expected = expected;
      this.found = found;
    }
  }

  /** An exception thrown when attempting to get a {@link ParsedRepr.Value Value} as a type that it isn't */
  public static class TypeMismatchException extends Exception 
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

  public static class GeneralException extends Exception
  {
    public final Serializable[] msg;

    public GeneralException(Serializable... msg) {this.msg = msg;}
  }

  /** 
   * A value with an attached type <p>
   * The value itself is stored as an Object, which is cast appropriately depending on the type.
   * It will therefore cause a runtime exception if: <ul>
   * <li> A value of type Num does not contain a {@code Double}
   * <li> A value of type Bool does not contain a {@code Boolean}
   * <li> A value of type String does not contain a {@code String}
   */
  public record Value(Type type, Object value) implements Serializable
  {
    /** The possible types of a value */
    public enum Type 
    {
      /** Number type, represented as a double */
      Num,
      /** String type, limited to {@code a..z}, {@code 0..9}, and {@code _}. Cannot start with a digit */
      Text
    }

    /** @return The underlying double value, or throws a {@link ParsedRepr.TypeMismatchException TypeMismatchException} if this value is not a Num */
    public double asNum() throws TypeMismatchException
    {
      return switch (type)
      {
        case Num -> ((Double)value).doubleValue();
        default -> throw new TypeMismatchException(Type.Num, this);
      };
    }

    /** 
     * @return The text treated as a boolean value, or throws a {@link ParsedRepr.TypeMismatchException TypeMismatchException} if this value is not Text.
     * Produces a warning if the value is Text, but not {@code on} or {@code off}
     */
    public boolean asBool() throws TypeMismatchException
    {
      return switch (type)
      {
        case Text -> switch ((String)value)
        {
          case "on" -> true;
          case "off" -> false;
          default -> 
          {
            AutoBuilder.error("warning: value ", value, " should be `on` or `off` (treated it as false/`off`)");
            yield false;
          }
        };
        default -> throw new TypeMismatchException(Type.Text, this);
      };
    }

    /** @return The underlying String value, or throws a {@link ParsedRepr.TypeMismatchException TypeMismatchException} if this value is not Text */
    public String asText() throws TypeMismatchException
    {
      return switch (type)
      {
        case Text -> (String)value;
        default -> throw new TypeMismatchException(Type.Text, this);
      };
    }

    @Override
    public final String toString() 
    {
      return value.toString() + ": " + type.toString();
    }
  }

  /** An instruction, formed of a instruction type and an array of {@link ParsedRepr.Value Values} */
  public record Instruction(Type type, Value... args) 
  {
    /** The possible types of instruction. 
     * Must all be lowercase, as the parser converts a lowercase String to this enum via the built-in {@link Enum#valueOf valueOf} method 
     */
    public enum Type
    {
      driveto, driveby, follow, 
      wait, waitfor, waituntil, 
      intake, 
      passing,
      climb
    }

    /** @return The argument at index {@code i} */
    public Value arg(int i)
      {return args[i];}

    /**
     * Throws an exception if the number of arguments in this instruction is different from the provided value.
     * Used as a helper to allow ergonomic, one-line checking of argument counts
     * @param count How many arguments the instruction should have
     * @throws ArgCountException
     */
    public void assertArgCount(int count) throws ArgCountException
    {
      if (args().length != count) 
        throw new ArgCountException(count, args().length);
    }

    @Override
    public final String toString() 
    {
      return type.toString() + Arrays.toString(args);
    }
  }
}
