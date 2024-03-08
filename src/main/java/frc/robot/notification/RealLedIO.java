package frc.robot.notification;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;

public class RealLedIO implements ILedIO{
  PWMSparkMax ledController = new PWMSparkMax(Constants.LEDController.LED_CONTROLLER_CHANNEL); 

  public RealLedIO(){}
  
  @Override
  public void changeColor(LEDState color){
    ledController.set(color.getSparkValue());  
  }

  public boolean getIsAlive(){
    return ledController.isAlive();
  }

}
