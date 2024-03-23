package frc.robot.auto.cmd.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;

/**
 * Move the arm to the home position. This *is* a blocking command.
 */
public class AutoArmHome extends Command {
    
    private final ArmSubsystem arm;
    private final Debouncer debouncer = new Debouncer(Constants.Arm.DEBOUNCE_TIME);

    public AutoArmHome(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.setTarget(Constants.Arm.HOME_POSITION);
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(arm.isAtHomePostion());
    }

}
