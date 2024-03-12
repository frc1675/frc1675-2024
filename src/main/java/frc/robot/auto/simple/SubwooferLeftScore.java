package frc.robot.auto.simple;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.cmdGroup.IntakeNote;
import frc.robot.cmdGroup.SpinUpAndShoot;
import frc.robot.drive.DriveSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.RobotContext;

public class SubwooferLeftScore extends SequentialCommandGroup {

    public SubwooferLeftScore(DriveSubsystem drive, ShooterSubsystem shooter, UndertakerSubsystem undertaker, RobotContext context) {
        addCommands(
            new AutoDriveReset(drive, Constants.Field.SUBWOOFER_LEFT),
            new SpinUpAndShoot(shooter, () -> false),
            new ParallelDeadlineGroup(
                new SimpleAutoDrive(drive, 4, true, -1), 
                new IntakeNote(shooter, undertaker, context::getReadyToIntake)
            )
        );
    }
}
