package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class SpinUpAndShoot extends SequentialCommandGroup {
    public SpinUpAndShoot(ShooterSubsystem shooter, DoubleSupplier topSpeed, DoubleSupplier bottomSpeed) {
        addCommands(
                new SpinUp(shooter, topSpeed, bottomSpeed),
                new Shoot(shooter).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME));
    }
}
