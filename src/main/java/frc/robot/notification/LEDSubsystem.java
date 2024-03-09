package frc.robot.notification;

import java.util.Stack;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase { 
  private final Stack<LEDState> stack = new Stack<LEDState>();

  private final ILedIO ledIO;

  public LEDSubsystem(ILedIO ledIO){
    this.ledIO = ledIO; 
    stack.push(LEDState.NOTHING);

    ledIO.changeColor(stack.peek());

    ShuffleboardTab tab = Shuffleboard.getTab("LEDStatus");
    tab.addBoolean("Connection", () -> ledIO.getIsAlive());
    tab.addString("Status", () -> stack.peek().getStatusMessage());
  }

  public void addColor(LEDState status){
    stack.push(status);
    status.setTimeAdded(Timer.getFPGATimestamp());
  }

  @Override
  public void periodic() {

    for (int i = 0; i < stack.size(); i++) {
      if (stack.get(i).isExpired(Timer.getFPGATimestamp())) {

        System.out.println(stack.remove(i).getStatusMessage());
        i--;
      }
    }

    ledIO.changeColor(stack.peek());

  }
 
}
