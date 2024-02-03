package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SpinUpAndShoot extends SequentialCommandGroup {
    public SpinUpAndShoot(ShooterSubsystem subsystem) {
        addCommands(
            new SpinUp(subsystem, 200),
            new Shoot(subsystem, 200)
        );
    }
}

