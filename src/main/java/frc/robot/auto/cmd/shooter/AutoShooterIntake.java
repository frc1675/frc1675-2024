package frc.robot.auto.cmd.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;

/**
 * Run the indexer at the given speed. Will stop when a note is loaded. This command will never end.
 */
public class AutoShooterIntake extends Command {
    
    private final ShooterSubsystem shooter;
    private final double speed;

    public AutoShooterIntake(ShooterSubsystem shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (shooter.isIndexerLoaded()) {
            shooter.setIndexerSpeed(0);
        }else {
            shooter.setIndexerSpeed(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
