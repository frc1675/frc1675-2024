package frc.robot.auto.cmd.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class NoteBlocker extends Command {
    
    private final ShooterSubsystem shooter;
    private final Timer timer;

    public NoteBlocker(ShooterSubsystem shooter) {
        this.shooter = shooter;

        timer = new Timer();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean inter) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() < Constants.Auto.CANCELLED_PATH_TRAVEL_TIME) {
            return false;
        }
        return shooter.isIndexerLoaded();
    }

}
