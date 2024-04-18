package frc.robot.notification;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.util.RobotContext;

public class ContextualColor extends Command {

  private final RobotContext robotContext;
  private final LEDSubsystem led;
  private final GenericHID toRumble;

  private final boolean[] previousValues = new boolean[5];

  public ContextualColor(RobotContext robotContext, LEDSubsystem led, GenericHID toRumble) {
    this.robotContext = robotContext;
    this.led = led;
    this.toRumble = toRumble;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    setPreviousValues();
  }

  @Override
  public void execute() {
    if (previousValues[0] != robotContext.hasNote()) {
      if (robotContext.hasNote()) {
        led.addColor(LEDState.HAS_NOTE);
        CommandScheduler.getInstance()
            .schedule(new RumbleController(toRumble).withTimeout(Constants.Controller.RUMBLE_TIME));
      } else {
        led.addColor(LEDState.SHOT_FIRED);
      }
    }

    handleContinuousEvent(
        previousValues[2], robotContext.getIntakeEnabledOverride(), LEDState.UNDERTAKER_DISABLED);
    handleContinuousEvent(
        previousValues[3], robotContext.shouldSlowShoot(), LEDState.AT_AMP_POSITION);

    setPreviousValues();
  }

  private void handleContinuousEvent(boolean previous, boolean current, LEDState state) {
    if (previous != current) {
      if (current) {
        state.requestRemoval();
      } else {
        led.addColor(state);
      }
    }
  }

  private void setPreviousValues() {
    previousValues[0] = robotContext.hasNote();
    previousValues[1] = false;
    previousValues[2] = robotContext.getIntakeEnabledOverride();
    previousValues[3] = robotContext.shouldSlowShoot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
