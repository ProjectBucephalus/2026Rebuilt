package frc.robot.util.autobuilder;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

public class Tokeniser 
{
  private final String source;
  private final ArrayList<Token> tokens = new ArrayList<>();

  private int start = 0;
  private int current = 0;

  public Tokeniser(String input)
  {
    source = input.toLowerCase();
  }

  public List<Token> tokenise()
  {
    while (!atEnd())
    {
      start = current;
      nextToken();
    }

    tokens.add(new Token(Token.Type.Eof, ""));

    return tokens;
  }

  private void nextToken()
  {
    char c = advance();
    switch (c)
    {
      case ' ', '\t', '\n', '\r' -> {}
      case ',' -> addToken(Token.Type.Comma);
      case '(' -> addToken(Token.Type.LParen);
      case ')' -> addToken(Token.Type.RParen);
      default -> 
      {
        if (c == '-' || isDigit(c)) 
          number();
        else if (isAlpha(c))
          text();
        else
          AutoBuilder.error("unexpected character " + c);
      }
    }
  }

  private void number() 
  {
    while (match(this::isDigit));

    // Look for a fractional part.
    if (match('.')) 
      while (match(this::isDigit));

    addToken(Token.Type.Num);
  }

  private void text() 
  {
    while (match(this::isAlphaNumeric));

    String text = source.substring(start, current);
    Token.Type type = switch (text) 
    {
      case "on", "off" -> Token.Type.Bool;
      default -> Token.Type.Text; 
    };
    addToken(type);
  }

  private boolean isDigit(char c)
    {return c >= '0' && c <= '9';}

  private boolean isAlpha(char c) 
  {
    return (c >= 'a' && c <= 'z') || c == '_';
  }

  private boolean isAlphaNumeric(char c) 
    {return isAlpha(c) || isDigit(c);}

  private boolean atEnd()
    {return current >= source.length();}

  private char advance() 
    {return source.charAt(current++);}

  private char peek() 
  {
    return atEnd() ? '\0' : source.charAt(current);
  }

  private boolean match(char expected) 
  {
    if (peek() != expected) return false;

    current++;
    return true;
  }

  private boolean match(Predicate<Character> cond)  
  {
    if (!cond.test(peek())) return false;

    current++;
    return true;
  }

  private void addToken(Token.Type type) 
  {
    tokens.add(new Token(type, source.substring(start, current)));
  }
}
