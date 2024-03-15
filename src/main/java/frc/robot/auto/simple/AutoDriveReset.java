package frc.robot.auto.simple;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.DriveSubsystem;

public class AutoDriveReset extends Command {
    
    private final Pose2d pose;
    private final DriveSubsystem drive;

    public AutoDriveReset(DriveSubsystem drive, Pose2d pose) {
        this.pose = pose;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.zeroGyroscope(pose.getRotation().getDegrees());
        drive.resetOdometry(pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
