package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.VisionSubsystem.LEDMode;

public interface IVision {

    public Pose2d getBotpose();

    public int getTargetId();

    public boolean hasTarget();

    public LEDMode getLEDMode();

    public void setLEDMode(LEDMode mode);
}
