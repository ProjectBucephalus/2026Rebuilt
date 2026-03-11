package frc.robot.util.autobuilder;

import java.util.ArrayList;
import java.util.List;

import frc.robot.util.autobuilder.Token.TokenType;

public class Lexer 
{
  private final String source;
  private final List<Token> tokens = new ArrayList<>();

  private int start = 0;
  private int current = 0;

  public Lexer(String input)
  {
    source = input;
  }

  public List<Token> tokenise()
  {
    while (!atEnd())
    {
      start = current;
      nextToken();
    }

    tokens.add(Token.eof);

    return tokens;
  }

  private void nextToken()
  {
    char c = advance();
    switch (c)
    {
      case ' ', '\t', '\n', '\r' -> {}
      case '"' -> string();

      default -> AutoBuilder.error(current, "unexpected character " + c);
    }
  }

  private void string()
  {
    while (!atEnd() && !match('"'));

    if (atEnd()) 
      AutoBuilder.error(current, "unterminated string");
    else
      addToken(TokenType.STRING);
  }

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

  private void addToken(TokenType type) 
  {
    tokens.add(new Token(type, source.substring(start, current)));
  }
}
