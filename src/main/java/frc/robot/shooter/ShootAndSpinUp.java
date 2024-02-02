package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootAndSpinUp extends SequentialCommandGroup {
    public ShootAndSpinUp(ShooterSubsystem subsystem) {
        addCommands(
            new SpinUpCommand(subsystem, 200),
            new ShootCommand(subsystem, 200)
        );
    }
}

