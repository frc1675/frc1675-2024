package frc.robot.auto.cmd.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;

/**
 * Spin up the shooter. This command *is* blocking.
 */
public class AutoSpinUp extends Command{
    
    private final ShooterSubsystem shooter;
    private final double topSpeed;
    private final double bottomSpeed;

    public AutoSpinUp(ShooterSubsystem shooter, double topSpeed, double bottomSpeed) {
        this.shooter = shooter;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetShooterSpeeds(topSpeed, bottomSpeed);
    }

    @Override
    public boolean isFinished() {
        return shooter.isShooterReady();
    }

}
