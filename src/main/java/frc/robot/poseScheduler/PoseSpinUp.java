package frc.robot.poseScheduler;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class PoseSpinUp extends PoseCommand {

    // will require shooter subsystem
    private Pose2d previous = null;

    @Override
    public void initialize() {
        System.out.println("I just spun up the shooter wheels!");
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean shouldSchedule(Pose2d currentPose) {
        if (previous != null && previous.getX() < Constants.Field.SCORING_AREA_X_BOUNDARY) {
            // We don't want to fire this command over and over, because it could interrupt
            // actual shooting.
            // Instead, we only want to call it when we first enter the important area.
            // Therefore, if we were in the important area last loop, we can assume that the
            // command was already called and we do not need to do so again.
            previous = currentPose;
            return false;
        }
        previous = currentPose;
        // We don't care about the y position of the robot for this example
        // We only care if the robot has entered the friendly side of the field
        return currentPose.getX() < Constants.Field.SCORING_AREA_X_BOUNDARY;
    }

}
