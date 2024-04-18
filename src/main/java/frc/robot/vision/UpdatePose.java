package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.DriveSubsystem;

public class UpdatePose extends Command {

  private final VisionSubsystem vision;
  private final DriveSubsystem drive;

  public UpdatePose(VisionSubsystem vision, DriveSubsystem drive) {
    addRequirements(vision);
    this.vision = vision;
    this.drive = drive;
  }

  @Override
  public void execute() {
    if (vision.hasTarget()) {
      drive.addVisionMeasurement(vision.getBotpose());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
