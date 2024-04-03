package frc.robot.auto.cmd.group;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto.cmd.arm.AutoArmMove;
import frc.robot.auto.cmd.shooter.AutoShoot;
import frc.robot.notification.AddLEDColor;
import frc.robot.notification.LEDState;
import frc.robot.notification.LEDSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;

public class ConfigurableShootSequence extends SequentialCommandGroup {

    public ConfigurableShootSequence(ShooterSubsystem shooter, UndertakerSubsystem undertaker, ArmSubsystem arm, LEDSubsystem led, DoubleSupplier armAngle) {
        addCommands(
            new AutoIntakeNote(shooter, undertaker).withTimeout(Constants.Auto.INTAKE_ATTEMPT_TIMEOUT),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new AutoArmMove(arm, armAngle),
                    new AutoShoot(shooter).withTimeout(Constants.Auto.SHOOT_TIME),
                    new InstantCommand( () -> {
                        DataLogManager.log("Did shot");
                    })
                ),
                new SequentialCommandGroup(
                    new AddLEDColor(led, LEDState.AUTONOMOUS_INTAKE_FAILED),
                    new InstantCommand( () -> {
                        DataLogManager.log("Detected that autonomous failed to intake note. Skipping shooting sequence.");
                    })
                ),
                shooter::isIndexerLoaded
            )
        );
    }
}
