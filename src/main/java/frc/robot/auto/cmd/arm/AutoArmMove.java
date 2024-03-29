package frc.robot.auto.cmd.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;

/* Move the arm to the given position. This *is* a blocking command. */
public class AutoArmMove extends Command {
    
    private final ArmSubsystem arm;
    private final DoubleSupplier position;
    private final Debouncer debouncer = new Debouncer(Constants.Arm.DEBOUNCE_TIME);

    public AutoArmMove(ArmSubsystem arm, DoubleSupplier position) {
        this.arm = arm;
        this.position = position;
    }

    public AutoArmMove(ArmSubsystem arm, double position) {
        this.arm = arm;
        this.position = () -> position;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        DataLogManager.log("Moving arm to " + position.getAsDouble());
        arm.setTarget(position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(arm.isOnTarget());
    }

    @Override
    public void end(boolean interrupted) {
        DataLogManager.log("Moved arm to " + position.getAsDouble());
    }

}
