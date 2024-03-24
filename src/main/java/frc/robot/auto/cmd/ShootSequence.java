package frc.robot.auto.cmd;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto.cmd.arm.AutoArmMove;
import frc.robot.auto.cmd.shooter.AutoShoot;
import frc.robot.auto.generator.AutonomousContext;
import frc.robot.notification.AddLEDColor;
import frc.robot.notification.LEDState;
import frc.robot.notification.LEDSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;

public class ShootSequence extends SequentialCommandGroup {

    public ShootSequence(ShooterSubsystem shooter, UndertakerSubsystem undertaker, ArmSubsystem arm, LEDSubsystem led, AutonomousContext context) {
        addCommands(
            new AutoIntakeNote(shooter, undertaker).withTimeout(Constants.Auto.INTAKE_ATTEMPT_TIMEOUT),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new AutoArmMove(arm, context::getAndIncrement),
                    new AutoShoot(shooter).withTimeout(Constants.Auto.SHOOT_TIME)
                ),
                new SequentialCommandGroup(
                    new AddLEDColor(led, LEDState.AUTONOMOUS_INTAKE_FAILED),
                    new InstantCommand( () -> {
                        DataLogManager.log("Detected that autonomous failed to intake note. Skipping shooting sequence.");
                        context.getAndIncrement(); //Increment the arm angle so that the angle is correct for the next shooting attempt
                    })
                ),
                shooter::isIndexerLoaded
            )
        );
    }
}
