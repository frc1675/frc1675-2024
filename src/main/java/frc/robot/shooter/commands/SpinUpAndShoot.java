package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class SpinUpAndShoot extends SequentialCommandGroup {
    public SpinUpAndShoot(ShooterSubsystem subsystem) {
        addCommands(
            new SpinUp(subsystem),
            new Shoot(subsystem).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME),
            new SpinDown(subsystem)
        );
    }
}
