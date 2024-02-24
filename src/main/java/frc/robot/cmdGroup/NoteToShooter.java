package frc.robot.cmdGroup;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.ShooterIntake;
import frc.robot.undertaker.IntakeNote;
import frc.robot.undertaker.UndertakerSubsystem;

public class NoteToShooter extends ParallelRaceGroup {
    public NoteToShooter(ShooterSubsystem shooter, UndertakerSubsystem under) {
        addCommands(
            new IntakeNote(under),
            new ShooterIntake(shooter, Constants.Undertaker.INTAKE_SPEED * .10) // run indexer at 10% of undertaker speed
        );
    }
}
