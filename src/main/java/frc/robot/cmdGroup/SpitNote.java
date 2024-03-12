package frc.robot.cmdGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.EjectShooter;
import frc.robot.undertaker.EjectNote;
import frc.robot.undertaker.UndertakerSubsystem;

public class SpitNote extends SequentialCommandGroup  {
    
    public SpitNote(ShooterSubsystem shooter, ArmSubsystem arm, UndertakerSubsystem undertaker) {
        addCommands(
            new MoveToPosition(arm, Constants.Arm.EJECT_POSITION),
            new ParallelCommandGroup(
                new EjectNote(undertaker),
                new EjectShooter(shooter)
            ).withTimeout(1),
            new MoveToHome(arm)
        );
    }

}
