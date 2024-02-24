package frc.robot.cmdGroup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.TrajectoryDrive;
import frc.robot.util.AutoGenerator;

public class SpeakerScore extends ConditionalCommand {
    public SpeakerScore(DriveSubsystem drive, AutoGenerator auto) {
        super( 
            new SequentialCommandGroup(
                new PrintCommand("Spinning up the motors placeholder"),
                new TrajectoryDrive(auto, drive, Rotation2d.fromDegrees(90), new Pose2d(Constants.Field.SPEAKER_SCORING_POSITION, Rotation2d.fromDegrees(0))),
                new PrintCommand("Moving the arm placeholder"),
                new PrintCommand("Shooting the note placeholder")
            ),
            new InstantCommand(() -> {
                DataLogManager.log("Too far away to dynamically path find to given pose. (Current Pose: " + drive.getPose()+")");
            }), 
            () -> auto.canFollowPath(new Pose2d(Constants.Field.SPEAKER_SCORING_POSITION, Rotation2d.fromDegrees(0)))
        );

    }

}
