package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.VisionSubsystem.LEDMode;

public class SimVision implements IVision {

    private Pose2d botPose = new Pose2d(0, 0, new Rotation2d(0));
    private boolean hasTarget = false;
    private int targetId = 0;
    private LEDMode currentMode;

    public SimVision() {}

    @Override
    public Pose2d getBotpose() {
        return botPose;
    }

    public void setBotPose(Pose2d targetPose) {
        this.botPose = targetPose;
    }

    @Override
    public boolean hasTarget() {
        return hasTarget;
    }

    public void setHasTarget(boolean value) {
        hasTarget = value;
    }

    @Override
    public int getTargetId() {
        return targetId;
    }

    public void setTargetId(int id) {
        targetId = id;
    }

    @Override
    public LEDMode getLEDMode() {
        return currentMode;
    }

    @Override
    public void setLEDMode(LEDMode mode) {
        currentMode = mode;
    }

    @Override
    public Rotation2d getTargetHorizontalOffset() {
        return null;
    }

    @Override
    public Rotation2d getTargetVerticalOffset() {
        return null;
    }

    @Override
    public boolean hasSpeaker() {
        return false;
    }

	@Override
	public double getHorizontalTranslation() {
	    return 0.0;
	}

}
