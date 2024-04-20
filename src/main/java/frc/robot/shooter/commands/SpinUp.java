package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class SpinUp extends Command {
    private final ShooterSubsystem subsystem;

    private final DoubleSupplier targetTopSpeed;
    private final DoubleSupplier targetBottomSpeed;

    public SpinUp(ShooterSubsystem subsystem, double targetTopSpeed, double targetBottomSpeed) {
        this.subsystem = subsystem;
        this.targetTopSpeed = () -> targetTopSpeed;
        this.targetBottomSpeed = () -> targetBottomSpeed;
        addRequirements(subsystem);
    }

    public SpinUp(ShooterSubsystem subsystem, DoubleSupplier targetTopSpeed, DoubleSupplier targetBottomSpeed) {
        this.subsystem = subsystem;
        this.targetTopSpeed = targetTopSpeed;
        this.targetBottomSpeed = targetBottomSpeed;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setTargetShooterSpeeds(targetTopSpeed.getAsDouble(), targetBottomSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return subsystem.isShooterReady();
    }
}
