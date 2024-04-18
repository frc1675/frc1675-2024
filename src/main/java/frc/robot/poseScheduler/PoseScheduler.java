package frc.robot.poseScheduler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.AbstractMap.SimpleEntry;
import java.util.HashMap;
import java.util.Map.Entry;

public class PoseScheduler {

  private HashMap<FieldArea2d, Entry<Command, Boolean>> commands =
      new HashMap<FieldArea2d, Entry<Command, Boolean>>();

  /**
   * Register a new command to be scheduled based on the robots current pose.
   *
   * @param c the command to register.
   */
  public void registerCommand(FieldArea2d area, Command c) {
    commands.put(area, new SimpleEntry<Command, Boolean>(c, false));
  }

  /**
   * This method should be called once every scheduler run (in the periodic method) in order to
   * schedule the commands appropriately.
   *
   * @param currentPose the current pose of the robot.
   */
  public void updatePose(Pose2d currentPose) {

    for (Entry<FieldArea2d, Entry<Command, Boolean>> pairs : commands.entrySet()) {
      if (pairs.getKey().withinArea(currentPose)) {
        if (!pairs.getValue().getValue()) {

          pairs.getValue().getKey().schedule();
        }
        pairs.getValue().setValue(true);
      } else {
        pairs.getValue().setValue(false);
      }
    }
  }
}
