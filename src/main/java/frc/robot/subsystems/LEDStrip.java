package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.leds.Block;

import java.util.ArrayList;
import java.util.List;

public class LEDStrip extends SubsystemBase
{
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final ArrayList<Block> blocks;
  private final double brightness;

  public LEDStrip(int pwmPort, double brightness, int length)
  {
    leds = new AddressableLED(pwmPort);    
    buffer = new AddressableLEDBuffer(length);
    blocks = new ArrayList<>();
    this.brightness = brightness;

    leds.setLength(length);
    leds.start();
  }

  public LEDStrip(int pwmPort, int length)
  {
    leds = new AddressableLED(pwmPort);    
    buffer = new AddressableLEDBuffer(length);
    blocks = new ArrayList<>();
    this.brightness = 0.2;

    leds.setLength(length);
    leds.start();
  }

  public LEDStrip(int pwmPort, double brightness, Block... blocks)
  {
    int length = 0;
    for (var block : blocks) length += block.length;

    leds = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(length);
    this.blocks = new ArrayList<>(List.of(blocks));
    this.brightness = brightness;

    leds.setLength(length);
    leds.start();
  }

  public LEDStrip(int pwmPort, Block... blocks)
  {
    int length = 0;
    for (var block : blocks) length += block.length;

    leds = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(length);
    this.blocks = new ArrayList<>(List.of(blocks));
    this.brightness = 0.2;

    leds.setLength(length);
    leds.start();
  }

  public Block getBlock(int pos)
    {return blocks.get(pos);}

  public void addBlock(Block block)
    {blocks.add(block);}

  public void addBlock(Block block, int pos)
    {blocks.add(pos, block);}

  public void removeBlock()
    {blocks.remove(blocks.size() - 1);}

  public void removeBlock(int pos)
    {blocks.remove(pos);}

  @Override
  public void periodic()
  {
    int currentPos = 0;
    for (var block : blocks) 
    {
      block.render(buffer, currentPos, brightness);
      currentPos += block.length;
    }

    leds.setData(buffer);
  }
}
