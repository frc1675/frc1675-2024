package frc.robot.notification;

import java.util.Stack;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase { 
  private final ShuffleboardTab tab;

  private final Stack<LEDState> stack = new Stack<LEDState>();

  private final ILedIO ledIO;

  public LEDSubsystem(ILedIO ledIO){
    this.ledIO = ledIO; 
    stack.push(LEDState.NOTHING);

    ledIO.changeColor(stack.get(0));

    tab = Shuffleboard.getTab("LEDStatus");
    tab.addBoolean("Connection", () -> ledIO.getIsAlive());
    tab.addString("Status:", () -> stack.get(0).getStatusMessage());
  }

  public void addColor(LEDState status){
    stack.push(status);
    status.setTimeAdded(Timer.getFPGATimestamp());
  }

  @Override
  public void periodic() {
    if (stack.peek().isExpired(Timer.getFPGATimestamp())) {
      stack.pop();
    }

    ledIO.changeColor(stack.peek());

  }
 
}
