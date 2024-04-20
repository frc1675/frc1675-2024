package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;

public class Shoot extends Command {
    private ShooterSubsystem subsystem;

    public Shoot(ShooterSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setIndexerSpeed(1);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setIndexerSpeed(0);
        subsystem.setTargetShooterSpeeds(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
