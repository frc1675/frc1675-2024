package frc.robot.vision;

import java.util.HashMap;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {

  public static enum LEDMode {
    OFF,
    BLINK,
    ON;
  }

  private static HashMap<Double, Double> angleToSpeakerDist = new HashMap<Double, Double>() {{
    put(8.9, 9.0);
  }};

  public static double getDistanceToSpeaker(double angleToTag) {
    // double lowMatch = -1000;
    // double highMatch = 1000;

    // for (Double angle : angleToSpeakerDist.keySet()) {
    //     if (angle >= lowMatch && angle <= angleToTag) {
    //         lowMatch = angle;
    //     } else if (angle <= highMatch && angle >= angleToTag) {
    //         highMatch = angle;
    //     }
    // }

    // double interpolant = MathUtil.inverseInterpolate(lowMatch, highMatch, angleToTag);
    // double dist = MathUtil.interpolate(angleToSpeakerDist.get(lowMatch), angleToSpeakerDist.get(highMatch), interpolant);

    return 2.0;
  }

  private final IVision visionImplementation;
  private final ShuffleboardTab tab;

  public static VisionSubsystem create() {
    return new VisionSubsystem(Robot.isReal() ? new RealVision() : new SimVision());
  }
    
  public VisionSubsystem(IVision implementation) {
    visionImplementation = implementation;
    tab = Shuffleboard.getTab("Vision");
    tab.addString("BotPose: ", () -> getBotpose().toString());
    tab.addBoolean("Has Speaker", () -> hasSpeaker());
    tab.addDouble("Horizontal Offset from Speaker", () -> getHorizontalSpeakerOffset() != null ? getHorizontalSpeakerOffset().getDegrees() : -1000);
    tab.addDouble("Vertical Offset from speaker", () -> getVerticalSpeakerOffset() != null ? getVerticalSpeakerOffset().getDegrees() : -1000);
    tab.addDouble("Displacement from speaker", () -> getDistanceToSpeaker() != null ? getDistanceToSpeaker().doubleValue() : -1000);
  }
  
  public boolean hasTarget(){
    return visionImplementation.hasTarget();
  }

  public Pose2d getBotpose(){
    Pose2d rtn = visionImplementation.getBotpose();
    if (rtn == null) {
      return new Pose2d();
    }
    return rtn;
  }

  public int getTargetId(){
    return visionImplementation.getTargetId();
  }

  public boolean hasSpeaker() {
    return visionImplementation.hasSpeaker();
  }

  public Rotation2d getHorizontalSpeakerOffset() {
    if (hasSpeaker())
      return visionImplementation.getTargetHorizontalOffset();
    return null;
  }

  public Rotation2d getVerticalSpeakerOffset() {
    if (hasSpeaker())
      return visionImplementation.getTargetVerticalOffset();
    return null;
  }

  public Double getDistanceToSpeaker() {
    return visionImplementation.getHorizontalDistance();
  }

  public LEDMode getLEDMode(){
    return visionImplementation.getLEDMode();
  }

  public void setLEDMode(LEDMode mode){
    visionImplementation.setLEDMode(mode);
  }
}
