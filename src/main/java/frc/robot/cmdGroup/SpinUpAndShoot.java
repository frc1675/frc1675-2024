package frc.robot.cmdGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.*;
import frc.robot.undertaker.UndertakerSubsystem;

public class SpinUpAndShoot extends SequentialCommandGroup {
    public SpinUpAndShoot(ShooterSubsystem shooter, UndertakerSubsystem undertaker) {
        addCommands(
            new SpinUp(shooter),
            new Shoot(shooter).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME),
            new IntakeNote(shooter, undertaker)
        );
    }
}
