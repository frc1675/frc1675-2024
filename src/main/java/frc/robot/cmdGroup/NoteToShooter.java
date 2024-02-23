package frc.robot.cmdGroup;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.ShooterIntake;
import frc.robot.shooter.commands.SpinUp;
import frc.robot.undertaker.IntakeNote;
import frc.robot.undertaker.UndertakerSubsystem;

public class NoteToShooter extends ParallelRaceGroup {
    public NoteToShooter(ShooterSubsystem shoot, UndertakerSubsystem under) {
        addCommands(
            new IntakeNote(under),
            new ShooterIntake(shoot, .3)
        );
    }
}
