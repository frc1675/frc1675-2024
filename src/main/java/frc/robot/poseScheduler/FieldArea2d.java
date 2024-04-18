package frc.robot.poseScheduler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Represents a rectangular area of the field in two dimensions. */
public class FieldArea2d {

  private final double xMin;
  private final double yMin;

  private final double xMax;
  private final double yMax;

  public FieldArea2d(double xMin, double yMin, double xMax, double yMax) {
    this.xMin = xMin;
    this.yMin = yMin;

    this.xMax = xMax;
    this.yMax = yMax;
  }

  public boolean withinArea(Translation2d pose) {
    return pose.getX() >= xMin && pose.getX() <= xMax && pose.getY() >= yMin && pose.getY() <= yMax;
  }

  public boolean withinArea(Pose2d pose) {
    return withinArea(pose.getTranslation());
  }
}
