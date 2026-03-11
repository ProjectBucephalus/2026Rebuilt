package frc.robot.util.autobuilder;

/** 
 * Represents a single Token extracted from the input text. Roughly analogous to a "word" 
 * @param type The type of the token, e.g. number or left parenthesis
 * @param text The original text corresponding to the token, used to extract values later on
 */
public record Token(Token.Type type, String text)
{
  /** Types of tokens */
  public static enum Type
  {
    /** {@code ,} */
    Comma, 
    /** {@code (} */
    LParen, 
    /** {@code )} */
    RParen,
    /** {@code hello_world}. Alphanumeric + underscores only. Case insensitive */
    Text, 
    /** {@code 1}, {@code -3}, {@code 3.14159} */
    Num, 
    /** {@code on} or {@code off} */
    Bool,
    /** Sentinel for end of input */
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