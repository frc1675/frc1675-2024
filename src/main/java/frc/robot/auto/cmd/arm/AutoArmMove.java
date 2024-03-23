package frc.robot.auto.cmd.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;

/* Move the arm to the given position. This *is* a blocking command. */
public class AutoArmMove extends Command {
    
    private final ArmSubsystem arm;
    private final double position;
    private final Debouncer debouncer = new Debouncer(Constants.Arm.DEBOUNCE_TIME);

    public AutoArmMove(ArmSubsystem arm, double position) {
        this.arm = arm;
        this.position = position;
    }

    @Override
    public void initialize() {
        arm.setTarget(position);
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(arm.isOnTarget());
    }

}
