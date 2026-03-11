package frc.robot.util.autobuilder;

import java.util.ArrayList;
import java.util.List;


import frc.robot.util.autobuilder.ParsedRepr.Instruction;
import frc.robot.util.autobuilder.ParsedRepr.Value;
import frc.robot.util.autobuilder.ParsedRepr.Value.Type;;

public class Parser 
{
  private final List<Token> source;
  private final ArrayList<Instruction> instrs = new ArrayList<>();

  private int pos = 0;

  public Parser(List<Token> tokens)
  {
    source = tokens;
  }

  public List<Instruction> parse()
  {
    while (peek() != Token.Type.Eof)
      nextInstr();

    return instrs;
  }

  private void nextInstr()
  {
    Token tok = advance();
    if (tok.type() != Token.Type.Text) 
    { 
      AutoBuilder.error("expected a name to begin instruction but found " + tok);
      skipPast(Token.Type.Comma);
      return;
    }
    String name = tok.text();

    tok = advance();
    if (tok.type() != Token.Type.LParen)
    {
      if (!(tok.type() == Token.Type.Comma || tok.type() == Token.Type.Eof))
      {
        AutoBuilder.error("expected `(` to begin arguments or `,` to end instruction but found " + tok);
        skipPast(Token.Type.Comma);
      }
      else addInstr(name);
      return;
    }
    
    ArrayList<Value> args = new ArrayList<>();
    loop: while (true)
    {
      tok = advance();
      switch (tok.type())
      {
        case Num -> args.add(new Value(Type.Num, Double.parseDouble(tok.text())));
        case Bool -> args.add(new Value(Type.Bool, tok.text()));
        case Text -> args.add(new Value(Type.String, tok.text()));
        case RParen -> {break loop;}
        default -> 
        {
          AutoBuilder.error("expected `)` to end arguments but found " + tok);
          skipPast(Token.Type.Comma);
          return;
        }
      }
    }

    tok = advance();
    if (!(tok.type() == Token.Type.Comma || tok.type() == Token.Type.Eof))
    {
      AutoBuilder.error("expected `,` to end instruction but found " + tok);
      skipPast(Token.Type.Comma);
      return;
    }
    
    addInstr(name, args.toArray(Value[]::new));
  }

  private Token advance() 
  {
    return pos >= source.size() ? new Token(Token.Type.Eof, "") : source.get(pos++);
  }

  private Token.Type peek() 
  {
    return pos >= source.size() ? Token.Type.Eof : source.get(pos).type();
  }

  private void skipPast(Token.Type target)
  {
    Token.Type tok = peek();
    while (!(tok == target || tok == Token.Type.Eof))
      tok = advance().type();

    advance();
  }

  private void addInstr(String name, Value... args) 
  {
    try 
    {
      instrs.add(new Instruction(Instruction.Type.valueOf(name), args));
    }
    catch (IllegalArgumentException e)
    {
      AutoBuilder.error(name + " is not a valid instruction");
    }
  }
}
