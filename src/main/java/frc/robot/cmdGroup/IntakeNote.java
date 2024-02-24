package frc.robot.cmdGroup;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.ShooterIntake;
import frc.robot.undertaker.UndertakerIntake;
import frc.robot.undertaker.UndertakerSubsystem;

public class IntakeNote extends ParallelRaceGroup {
    public IntakeNote(ShooterSubsystem shooter, UndertakerSubsystem undertaker) {
        addCommands(
            new UndertakerIntake(undertaker),
            new ShooterIntake(shooter) // run indexer at 10% of undertaker speed
        );
    }
}
