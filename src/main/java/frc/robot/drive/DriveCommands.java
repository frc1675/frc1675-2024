package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.AllianceUtil;

public class DriveCommands {

    public static Command turnToShootCommand(DriveSubsystem drive) {
        return new InstantCommand(
                () -> drive.setRotationTarget(Rotation2d.fromDegrees(AllianceUtil.isRedAlliance() ? 0 : 180)), drive);
    }

    public static Command turnToAmpCommand(DriveSubsystem drive) {
        return new InstantCommand(() -> drive.setRotationTarget(Rotation2d.fromDegrees(-90)), drive);
    }

    public static Command zeroGyroscope(DriveSubsystem drive) {
        return new InstantCommand(drive::zeroGyroscope, drive);
    }
}
