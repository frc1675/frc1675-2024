package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class EjectShooter extends Command {
    
    private final ShooterSubsystem shooter;

    public EjectShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.setIndexerSpeed(Constants.Shooter.EJECT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setIndexerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
