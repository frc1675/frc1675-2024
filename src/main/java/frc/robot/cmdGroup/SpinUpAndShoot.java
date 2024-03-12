package frc.robot.cmdGroup;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.*;

public class SpinUpAndShoot extends SequentialCommandGroup {
  public SpinUpAndShoot(ShooterSubsystem shooter, BooleanSupplier slowShoot) {
    addCommands(
        new SpinUp(shooter, Constants.Shooter.SHOOT_SPEED, slowShoot),
        new Shoot(shooter).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME));
  }
}
