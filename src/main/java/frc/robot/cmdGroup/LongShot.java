package frc.robot.cmdGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.Shoot;
import frc.robot.shooter.commands.SpinUp;

public class LongShot extends SequentialCommandGroup {
  public LongShot(ShooterSubsystem shooter, ArmSubsystem arm) {
    addCommands(
        new MoveToPosition(arm, Constants.Arm.LONG_SHOT_ANGLE),
        new SpinUp(shooter, Constants.Shooter.LONG_SHOT_SPEED, Constants.Shooter.LONG_SHOT_SPEED),
        new Shoot(shooter).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME));
  }
}
