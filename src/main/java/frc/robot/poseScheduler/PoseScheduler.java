package frc.robot.poseScheduler;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseScheduler {

    private List<PoseCommand> commands = new ArrayList<PoseCommand>();

    /**
     * Register a new command to be scheduled based on the robots current pose.
     * 
     * @param c the command to register.
     */
    public void registerCommand(PoseCommand c) {
        commands.add(c);
    }

    /**
     * This method should be called once every scheduler run (in the periodic
     * method) in order to schedule the commands appropriately.
     * 
     * @param currentPose the current pose of the robot.
     */
    public void updatePose(Pose2d currentPose) {
        for (PoseCommand c : commands) {
            if (c.shouldSchedule(currentPose)) {
                c.schedule();
            }
        }
    }

}
