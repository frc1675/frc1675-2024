package frc.robot.auto.cmd.group;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.cmd.shooter.AutoShooterIntake;
import frc.robot.auto.cmd.undertaker.AutoUndertaker;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;

/** Runs the undertaker and shooter indexer in intake mode. This command will never end. */
public class AutoIntakeNote extends ParallelCommandGroup {
  public AutoIntakeNote(ShooterSubsystem shooter, UndertakerSubsystem undertaker) {
    addCommands(
        new AutoUndertaker(undertaker, Constants.Undertaker.INTAKE_SPEED, shooter::isIndexerLoaded),
        new AutoShooterIntake(shooter, Constants.Shooter.INTAKE_SPEED));
  }
}
