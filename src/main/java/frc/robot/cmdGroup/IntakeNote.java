package frc.robot.cmdGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.ShooterIntake;
import frc.robot.undertaker.UndertakerIntake;
import frc.robot.undertaker.UndertakerSubsystem;
import java.util.function.BooleanSupplier;

public class IntakeNote extends ParallelCommandGroup {
    public IntakeNote(ShooterSubsystem shooter, UndertakerSubsystem undertaker, BooleanSupplier readyToIntake) {
        addCommands(
                new UndertakerIntake(undertaker, () -> (!shooter.isIndexerLoaded() && readyToIntake.getAsBoolean())),
                new ShooterIntake(shooter, readyToIntake));
    }
}
