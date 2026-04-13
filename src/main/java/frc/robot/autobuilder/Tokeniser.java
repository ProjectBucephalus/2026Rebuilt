package frc.robot.autobuilder;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/** Converts raw source text into a structured list of {@link Token Tokens}, handling any invalid characters */
public class Tokeniser 
{
  /** The source text */
  private final String source;
  /** The list that the tokens are placed into as they're read */
  private final ArrayList<Token> tokens = new ArrayList<>();

  /** The start index of the token currently being read */
  private int start = 0;
  /** The current position we're up to in the source. Never less than {@link Tokeniser#start start} */
  private int current = 0;

  /** @param input The source text for this tokeniser to read */
  public Tokeniser(String input)
  {
    source = input.toLowerCase();
  }

  /**
   * Runs the actual tokenising process, iterating through the source text and creating tokens as they are read
   * @return The list of tokens
   */
  public List<Token> tokenise()
  {
    // The null byte \0 is used to signal the end of the source text
    while (peek() != '\0')
    {
      // Set the start of the new token to our current position, then read the token
      start = current;
      nextToken();
    }

    return tokens;
  }

  /** Reads one token from the current position and adds it to the list */
  private void nextToken()
  {
    char c = advance();
    switch (c)
    {
      // Ignore whitespace
      case ' ', '\t', '\n', '\r' -> {}
      // Simple handling for single-character punctuation tokens
      case ',' -> addToken(Token.Type.Comma);
      default -> 
      {
        // Numbers can start with a negative sign or a digit
        if (c == '-' || isDigit(c))
          number();
        // Text must start with a..z or _
        else if (isAlpha(c))
          text();
        // Print an error if we don't recognise the character
        else
          AutoBuilder.error("unexpected character " + c);
      }
    }
  }

  /** Handles number tokens */
  private void number() 
  {
    // Consume all consecutive digits
    while (match(this::isDigit));

    // Look for a fractional part, consuming all consecutive digits after one
    if (match('.')) 
      while (match(this::isDigit));

    addToken(Token.Type.Num);
  }

  /** Handles text tokens */
  private void text() 
  {
    // Consume all consecutive alphanumeric characters
    while (match(this::isAlphaNumeric));

    addToken(Token.Type.Text);
  }

  /** Helper for checking if a character is 0..9 */
  private boolean isDigit(char c)
    {return c >= '0' && c <= '9';}

  /** 
   * Helper for checking if a character is a..z or _ <p>
   * Everything is handled in lowercase, so uppercase characters do not need to be considered
   */
  private boolean isAlpha(char c) 
  {
    return (c >= 'a' && c <= 'z') || c == '_';
  }

  /** Helper for checking if a character is 0..9, a..z, or _ */
  private boolean isAlphaNumeric(char c) 
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
   * Adds a new token to the list, using {@link Tokeniser#start start} and {@link Tokeniser#current current} to get it's source text
   * @param type The type of the new token
   */
  private void addToken(Token.Type type) 
  {
    tokens.add(new Token(type, source.substring(start, current)));
  }
}
