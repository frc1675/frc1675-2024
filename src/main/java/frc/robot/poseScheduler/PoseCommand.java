package frc.robot.poseScheduler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command which can be automatically scheduled based on the robots current
 * pose.
 */
public abstract class PoseCommand extends Command {

    /**
     * This method should return true if the command is requesting scheduling based
     * on the robots current pose.
     * 
     * @param currentPose The current pose of the robot
     * @return true if the command wants to be scheduled, false otherwise.
     */
    public abstract boolean shouldSchedule(Pose2d currentPose);
}
