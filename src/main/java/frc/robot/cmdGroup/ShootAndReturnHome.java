package frc.robot.cmdGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.SpinUpAndShoot;
import java.util.function.DoubleSupplier;

public class ShootAndReturnHome extends SequentialCommandGroup {

    public ShootAndReturnHome(
            ShooterSubsystem shooter, ArmSubsystem arm, DoubleSupplier topSpeed, DoubleSupplier bottomSpeed) {
        addCommands(new SpinUpAndShoot(shooter, topSpeed, bottomSpeed), new MoveToHome(arm));
    }
}
