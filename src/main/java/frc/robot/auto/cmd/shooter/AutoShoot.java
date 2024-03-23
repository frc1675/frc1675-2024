package frc.robot.auto.cmd.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;

/**
 * Spin the indexer motors at full speed in order to shoot the note. Shooter motors must be spun up before calling this command.
 * This command will never end, recommended usage is decorating with a <code>.withTimeout()</code>.
 */
public class AutoShoot extends Command{
    
    private final ShooterSubsystem shooter;

    public AutoShoot(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.setIndexerSpeed(1);
    }

    @Override
    public void end(boolean inter) {
        shooter.setIndexerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
