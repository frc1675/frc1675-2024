package frc.robot.notification;

import edu.wpi.first.wpilibj2.command.Command;

public class AddLEDColor extends Command {

  private final LEDState state;
  private final LEDSubsystem led;

  public AddLEDColor(LEDSubsystem led, LEDState state) {
    this.state = state;
    this.led = led;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    led.addColor(state);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
