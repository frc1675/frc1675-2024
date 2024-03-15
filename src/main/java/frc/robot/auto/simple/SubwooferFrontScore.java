package frc.robot.auto.simple;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.cmdGroup.IntakeNote;
import frc.robot.cmdGroup.PodiumShot;
import frc.robot.drive.DriveSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.SpinUpAndShoot;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.RobotContext;

public class SubwooferFrontScore extends SequentialCommandGroup {

    public SubwooferFrontScore(double delay, DriveSubsystem drive, ShooterSubsystem shooter, UndertakerSubsystem undertaker, ArmSubsystem arm, RobotContext context) {
        addCommands(
            new AutoDriveReset(drive, Constants.Field.SUBWOOFER_FRONT),
            new SpinUpAndShoot(shooter, () -> false),
            new WaitCommand(delay),
            new ParallelDeadlineGroup(
                new SimpleAutoDrive(drive, 4, true, -1), 
                new IntakeNote(shooter, undertaker, context::getReadyToIntake)
            ),
            new PodiumShot(shooter, arm)
        );
    }
}
