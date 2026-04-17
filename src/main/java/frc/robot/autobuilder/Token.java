package frc.robot.autobuilder;

/** 
 * Represents a single Token extracted from the input text. Roughly analogous to a "word" 
 * @param type The type of the token, e.g. number or left parenthesis
 * @param text The original text corresponding to the token, used to extract values later on
 */
public record Token(Token.Type type, String text)
{
  /** Types of tokens */
  public enum Type
  {
    /** {@code ,} */
    Comma, 
    /** {@code hello_world}. Alphanumeric + underscores only. Case insensitive */
    Text, 
    /** {@code 1}, {@code -3}, {@code 3.14159} */
    Num, 
    /** Sentinel for end of input */
    Eof;

    @Override
    public final String toString() 
    {
      return switch (this) 
      {
        case Comma -> "`,`";
        case Text -> "text";
        case Num -> "number";
        case Eof -> "end of input";
      };
    }
  }

  @Override
  public final String toString() 
  {
    return switch (this.type())
    {
      case Comma, Eof -> this.type().toString();
      case Text, Num -> '`' + this.text() + '`';
    };
  }
}