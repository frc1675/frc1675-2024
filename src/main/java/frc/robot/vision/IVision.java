package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface IVision {

    public Pose2d getBotpose();

    public int getTargetId();

    public boolean hasTarget();
}
