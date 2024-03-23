package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.VisionSubsystem.LEDMode;

public interface IVision {
  public Pose2d getBotpose();

  public int getTargetId();

  public boolean hasTarget();

  public LEDMode getLEDMode();

  public void setLEDMode(LEDMode mode);

  public boolean hasSpeaker();

  public Rotation2d getTargetHorizontalOffset();

  public Rotation2d getTargetVerticalOffset();

  public Double getHorizontalDistance();
}
