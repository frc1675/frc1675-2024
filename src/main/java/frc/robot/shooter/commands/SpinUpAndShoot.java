package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class SpinUpAndShoot extends SequentialCommandGroup {
    public SpinUpAndShoot(ShooterSubsystem subsystem) {
        addCommands(
            new SpinUp(subsystem, Constants.Shooter.TARGET_SHOOTER_SPEED),
            new Shoot(subsystem, Constants.Shooter.TARGET_INDEXER_SPEED).withTimeout(Constants.Shooter.WAIT_UNTIL_END_SECS)
        );
    }
}

