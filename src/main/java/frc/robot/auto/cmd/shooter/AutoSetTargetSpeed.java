package frc.robot.auto.cmd.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;

/**
 * Spin up the shooter. This command *is not* blocking. It will finish instantly.
 */
public class AutoSetTargetSpeed extends Command{
    
    private final ShooterSubsystem shooter;
    private final double topSpeed;
    private final double bottomSpeed;

    public AutoSetTargetSpeed(ShooterSubsystem shooter, double topSpeed, double bottomSpeed) {
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
        return true;
    }

}
