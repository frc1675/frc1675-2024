package frc.robot.notification;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SimLedIO implements ILedIO{

  private double ledSparkValue = 0;
  private final Mechanism2d uiLED;
  
  public SimLedIO(){
    uiLED = new Mechanism2d(3, 3);
  }

  @Override
  public void changeColor(LEDStateEnum color){
    ledSparkValue = color.getSparkValue();
    uiLED.setBackgroundColor(new Color8Bit(color.getHexCode()));
  }

  public double getLedSparkValue(){
    return ledSparkValue;
  }

  @Override
  public boolean getIsAlive(){
    return true;
  }

}