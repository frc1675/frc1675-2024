package frc.robot.cmdGroup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.TurnToAngle;
import frc.robot.vision.VisionSubsystem;

public class SpeakerAlign extends SequentialCommandGroup {
  public SpeakerAlign(DriveSubsystem drive, VisionSubsystem vision) {
    addCommands(
      new TurnToAngle(drive, getAngleToSpeaker(drive.getPose().getRotation(), vision.getHorizontalSpeakerOffset()))
    );
  }

  private double getAngleToSpeaker(Rotation2d currentDriveAngle, Rotation2d offset) {
    return currentDriveAngle.rotateBy(offset).getDegrees();
  }
}
