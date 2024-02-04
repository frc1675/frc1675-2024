package frc.robot.notification;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase{ 
  private final ShuffleboardTab tab;
  private final PWMSparkMax ledController = new PWMSparkMax(Constants.LEDController.LED_CONTROLLER_CHANNEL); 
  private LEDStateEnum status;

  public LEDSubsystem(){
    tab = Shuffleboard.getTab("LEDStatus");
    tab.addBoolean("Connection", () -> ledController.isAlive());
    tab.addString("Status:", () -> status.getStatusMessage());
  }

  public void handleStateUpdate(LEDStateEnum status){
    ledController.set(status.getSparkValue()); 
    this.status = status;
  }
 
}
