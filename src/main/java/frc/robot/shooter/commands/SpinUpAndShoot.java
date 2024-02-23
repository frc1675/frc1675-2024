package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class SpinUpAndShoot extends SequentialCommandGroup {
    public SpinUpAndShoot(ShooterSubsystem subsystem) {
        addCommands(
            new SpinUp(subsystem, .3).withTimeout(Constants.Shooter.WAIT_UNTIL_END_SECS), // TODO: remove first withTimeout after PID is setup
            new Shoot(subsystem, .3).withTimeout(Constants.Shooter.WAIT_UNTIL_END_SECS)
        );
    }
}

