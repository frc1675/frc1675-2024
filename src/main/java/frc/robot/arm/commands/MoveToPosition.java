package frc.robot.arm.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;

public class MoveToPosition extends Command {
    private ArmSubsystem arm;
    private double targetAngle;
    private Debouncer debouncer;

    public MoveToPosition(ArmSubsystem arm, double targetAngle) {
        this.targetAngle = targetAngle;
        this.arm = arm;
        debouncer = new Debouncer(Constants.Arm.DEBOUNCE_TIME);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTarget(targetAngle);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return debouncer.calculate(arm.isOnTarget());
    }
}
