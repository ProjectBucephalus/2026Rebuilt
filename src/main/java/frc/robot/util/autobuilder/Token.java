package frc.robot.util.autobuilder;

public record Token(TokenType type, String text)
{
  public static enum TokenType
  {
    IDENT, 
    NUM, BOOL, STRING,
    EOF
  }  

  public static final Token eof = new Token(TokenType.EOF, "");
}
