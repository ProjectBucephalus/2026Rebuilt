package frc.robot.autobuilder;

import java.util.Optional;
import java.util.function.Predicate;

/** Lazily converts raw source text into a series of {@link Token Tokens}, handling any invalid characters */
public class Tokeniser 
{
  /** The source text */
  private String source;
  /** The start index of the token currently being read */
  private int start;
  /** The current position we're up to in the source. Never less than {@link Tokeniser#start start} */
  private int current;

  /**
   * Resets the tokeniser, storing the new source text and resetting the indices <p>
   * Must be have been called at least once prior to any {@link Tokeniser#nextToken nextToken()} calls
   * @param source The source text for the tokeniser to read in future {@link Tokeniser#nextToken nextToken()} calls
   */
  public void reset(String source)
  {
    this.source = source.toLowerCase();
    start = 0;
    current = 0;
  }

  /** 
   * Produces the next token
   * @return The next token, or empty if the end of the source text has been reached
   */
  public Optional<Token> nextToken()
  {
    start = current;
    char c = advance();
    return switch (c)
    {
      // End of source text, return empty
      case '\0' -> Optional.empty();
      // Ignore whitespace
      case ' ', '\t', '\n', '\r' -> nextToken();
      // Simple handling for single-character punctuation tokens
      case ',' -> Optional.of(createToken(Token.Type.Comma));
      // Numbers can start with a negative sign or a digit
      case '-', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' -> Optional.of(number());
      default -> 
      {
        // Text must start with a..z or _
        if (isAlpha(c)) 
          yield Optional.of(text());
        // Print an error if we don't recognise the character
        else 
        {
          AutoBuilder.error("unexpected character " + c);
          yield nextToken();
        }
      }
    };
  }

  /** Handles number tokens */
  private Token number() 
  {
    // Consume all consecutive digits
    while (match(Tokeniser::isDigit));

    // Look for a fractional part, consuming all consecutive digits after one
    if (match('.')) 
      while (match(Tokeniser::isDigit));

    return createToken(Token.Type.Num);
  }

  /** Handles text tokens */
  private Token text() 
  {
    // Consume all consecutive alphanumeric characters
    while (match(Tokeniser::isAlphaNumeric));

    return createToken(Token.Type.Text);
  }

  /** Helper for checking if a character is 0..9 */
  private static boolean isDigit(char c)
    {return c >= '0' && c <= '9';}

  /** 
   * Helper for checking if a character is a..z or _ <p>
   * Everything is handled in lowercase, so uppercase characters do not need to be considered
   */
  private static boolean isAlpha(char c) 
  {
    return (c >= 'a' && c <= 'z') || c == '_';
  }

  /** Helper for checking if a character is 0..9, a..z, or _ */
  private static boolean isAlphaNumeric(char c) 
    {return isAlpha(c) || isDigit(c);}

  /** Gets the current character in the source and advances our position, or returns {@code '\0'} if we're already past the end of the source */
  private char advance() 
    {return current >= source.length() ? '\0' : source.charAt(current++);}

  /** Gets the current character in the source without advancing, or {@code '\0'} if we're already past the end of the source */
  private char peek() 
    {return current >= source.length() ? '\0' : source.charAt(current);}

  /**
   * Checks whether the current character matches a provided condition, advancing our position if it does
   * @param cond The condition to check the character against
   * @return True if the condition returns true
   */
  private boolean match(Predicate<Character> cond)  
  {
    if (!cond.test(peek())) return false;

    current++;
    return true;
  }

  /**
   * Checks whether the current character matches a provided value, advancing our position if it does
   * @param expected The desired character
   * @return True if the current character matches
   */
  private boolean match(char expected) 
    {return match(c -> c == expected);}

  /**
   * Creates a new token, using {@link Tokeniser#start start} and {@link Tokeniser#current current} to get it's source text
   * @param type The type of the new token
   * @return The new token
   */
  private Token createToken(Token.Type type) 
  {
    return new Token(type, source.substring(start, current));
  }
}
