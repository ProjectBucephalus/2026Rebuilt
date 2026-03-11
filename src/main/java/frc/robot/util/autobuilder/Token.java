package frc.robot.util.autobuilder;

  public record Token(Token.Type type, String text)
  {
    public static enum Type
    {
      Comma, LParen, RParen,
      Text, 
      Num, Bool,
      Eof;

      @Override
      public final String toString() 
      {
        return switch (this) 
        {
          case Comma -> "`,`";
          case LParen -> "`(`";
          case RParen -> "`)`";
          case Text -> "text";
          case Num -> "number";
          case Bool -> "`on`/`off`";
          case Eof -> "end of input";
        };
      }
    }

    @Override
    public final String toString() 
    {
      return switch (this.type())
      {
        case Comma, LParen, RParen, Eof -> this.type().toString();
        case Text, Num, Bool -> '`' + this.text() + '`';
      };
    }
  }