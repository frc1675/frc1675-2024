package frc.robot.notification;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class LEDSubsystem extends SubsystemBase { 
  private final ShuffleboardTab tab;
  private LEDStateEnum status = LEDStateEnum.RED;
  private final ILedIO ledIO;

  public LEDSubsystem(ILedIO ledIO){
    this.ledIO = ledIO; 
    tab = Shuffleboard.getTab("LEDStatus");
    tab.addBoolean("Connection", () -> ledIO.getIsAlive());
    tab.addString("Status:", () -> status.getStatusMessage());
  }

  public void changeColor(LEDStateEnum status){
    ledIO.changeColor(status);
    this.status = status;
  }
 
}
