package frc.robot.notification;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.RobotContext;

public class ContextualColor extends Command {
    
    private final RobotContext robotContext;
    private final LEDSubsystem led;
    private final boolean[] previousValues = new boolean[5];

    public ContextualColor(RobotContext robotContext, LEDSubsystem led) {
        this.robotContext = robotContext;
        this.led = led;
    }

    @Override
    public void initialize() {
        setPreviousValues();
    }

    @Override
    public void execute() {
        if (previousValues[0] != robotContext.hasNote() && robotContext.hasNote()) {
            led.addColor(LEDState.HAS_NOTE);
        }

        setPreviousValues();
    }

    private void setPreviousValues() {
        previousValues[0] = robotContext.hasNote();
    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}