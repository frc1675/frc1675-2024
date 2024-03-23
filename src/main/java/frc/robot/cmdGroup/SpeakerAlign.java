package frc.robot.cmdGroup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.TurnToAngle;
import frc.robot.vision.VisionSubsystem;

public class SpeakerAlign extends SequentialCommandGroup {
  public SpeakerAlign(DriveSubsystem drive, VisionSubsystem vision) {
    addCommands(
      new TurnToAngle(drive, () -> getAngleToSpeaker(drive, vision))
    );
  }

  private double getAngleToSpeaker(DriveSubsystem drive, VisionSubsystem vision) {
      double offset = vision.getHorizontalSpeakerOffset() != null ? vision.getHorizontalSpeakerOffset().getDegrees() : -1000;
      System.out.println(offset); 
      if (offset < 60)
        return drive.getPose().getRotation().getDegrees();
      return drive.getPose().getRotation().rotateBy(new Rotation2d(offset)).getDegrees();
  }
}
