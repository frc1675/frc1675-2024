package frc.robot.auto.cmd.group;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto.cmd.arm.AutoArmMove;
import frc.robot.auto.cmd.shooter.AutoShoot;
import frc.robot.notification.LEDSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;
import java.util.function.DoubleSupplier;

public class ConfigurableShootSequence extends SequentialCommandGroup {

  public ConfigurableShootSequence(
      ShooterSubsystem shooter,
      UndertakerSubsystem undertaker,
      ArmSubsystem arm,
      LEDSubsystem led,
      DoubleSupplier armAngle) {
    addCommands(
        new AutoIntakeNote(shooter, undertaker).withTimeout(Constants.Auto.INTAKE_ATTEMPT_TIMEOUT),
        new AutoArmMove(arm, armAngle),
        new AutoShoot(shooter).withTimeout(Constants.Auto.SHOOT_TIME),
        new InstantCommand(
            () -> {
              DataLogManager.log("Did shot");
            }));
  }
}
