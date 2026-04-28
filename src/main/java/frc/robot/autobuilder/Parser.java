package frc.robot.autobuilder;

import java.util.ArrayList;
import java.util.List;

import frc.robot.autobuilder.ParsedRepr.Instruction;
import frc.robot.autobuilder.ParsedRepr.Value;
import frc.robot.autobuilder.ParsedRepr.Value.Type;

/**
 * Converts {@link Token Tokens} into {@link ParsedRepr.Instruction Instructions}, 
 * discarding semantically meaningless punctuation and extracting literal values 
 */
public class Parser 
{
  private final Tokeniser tokeniser = new Tokeniser();
  /** The list that the instructions are placed into as they're parsed */
  private ArrayList<Instruction> instrs;

  /**
   * Runs the parsing process, iterating through the tokens and grouping them into instructions
   * @param tokens The tokens for the parser to parse
   * @return The list of parsed instructions
   */
  public List<Instruction> parse(String source)
  {
    tokeniser.reset(source);
    instrs = new ArrayList<>();

    while (!nextInstr());

    return instrs;
  }

  /** 
   * Parses one instruction starting at the current position and adds it to the list 
   * @return True if we have reached Eof and should stop parsing
   */
  private boolean nextInstr()
  {
    // Consume the next token, ensuring that it is Text (the name of this instruction)
    Token tok = advance();
    if (tok.type() != Token.Type.Text) 
    { 
      err("expected an instruction but found " + tok);
      return tok.type() == Token.Type.Eof;
    }
    String name = tok.text();
    
    // Iterate through tokens and construct a list of arguments from them
    ArrayList<Value> args = new ArrayList<>();
    boolean eof = false;
    loop: while (true)
    {
      tok = advance();
      switch (tok.type())
      {
        // Extract relevant values from literal tokens and add them to the argument list
        // Bools are stored as their literal `on` or `off` value, and converted into Java booleans when accessed. 
        // This is used for better diagnostics
        case Num -> args.add(new Value(Type.Num, Double.parseDouble(tok.text())));
        case Text -> args.add(new Value(Type.Text, tok.text()));
        // Exit the loop if we've reached the end of the instruction
        case Comma -> {break loop;}
        // Exit the loop if we've reached the end of the input, flagging that fact
        case Eof -> 
        {
          eof = true;
          break loop;
        }
      }
    }

    // Add the parsed instruction to the list
    addInstr(name, args);

    return eof;
  }

  /** 
   * Gets the current token in the source and advances our position, 
   * or returns a blank {@link Token.Type#Eof end of file} token if we're already past the end of the source 
   */
  private Token advance() 
  {
    return tokeniser.nextToken().orElseGet(() -> new Token(Token.Type.Eof, ""));
  }

  /**
   * Skips tokens until we have passed a token of provided type
   * @param target The token type to skip past
   */
  private void skipPast(Token.Type target)
  {
    // Consume tokens until we've consumed the target token type or we reach the end of the tokens
    Token.Type tok = advance().type();
    while (!(tok == target || tok == Token.Type.Eof))
      tok = advance().type();
  }

  /**
   * Helper to report an error and skip to the next instruction, ensuring we don't attempt to use an invalid instruction
   * @param message The error message to report
   */
  private void err(String message)
  {
    AutoBuilder.error(message);
    skipPast(Token.Type.Comma);
  }

  /**
   * Adds a new instruction to the list, reporting an error if the provided name is not a valid instruction type
   * @param name The name of the instruction to add
   * @param args The arguments to the instruction, which get converted from a List to an Array
   */
  private void addInstr(String name, List<Value> args) 
  {
    Instruction.Type instr;
    try {instr = Instruction.Type.valueOf(name);}
    catch (IllegalArgumentException e)
    {
      AutoBuilder.error(name + " is not a valid instruction");
      return;
    }
    instrs.add(new Instruction(instr, args.toArray(Value[]::new)));
  }
}
