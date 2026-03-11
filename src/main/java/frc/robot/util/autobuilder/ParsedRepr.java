package frc.robot.util.autobuilder;

import java.util.Arrays;

import frc.robot.util.autobuilder.ParsedRepr.Value.Type;

public final class ParsedRepr 
{
  private ParsedRepr() {}

  public static class TypeMismatchException extends RuntimeException 
  {
    public final Type expected;
    public final Value found;

    public TypeMismatchException(Type expected, Value found)
    {
      this.expected = expected;
      this.found = found;
    }
  }

  public static record Value(Type type, Object value)
  {
    public static enum Type 
    {
      Num,
      Bool,
      String
    }

    public double asNum()
    {
      return switch (type)
      {
        case Num -> ((Double)value).doubleValue();
        default -> throw new TypeMismatchException(Type.Num, this);
      };
    }

    public boolean asBool()
    {
      return switch (type)
      {
        case Bool -> value == "on" ? true : false; 
        default -> throw new TypeMismatchException(Type.Bool, this);
      };
    }

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

  public static record Instruction(Type type, Value... args) 
  {
    public static enum Type
    {
      driveto, follow, 
      waitfor, waituntil, 
      intake, 
      passing;
    }

    public Value arg(int i)
      {return args[i];}

    @Override
    public final String toString() 
    {
      return type.toString() + Arrays.toString(args);
    }
  }
}
