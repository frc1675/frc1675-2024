package frc.robot.vision;

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

  private final IVision visionImplementation;
  private final ShuffleboardTab tab;

  public static VisionSubsystem create() {
    return new VisionSubsystem(Robot.isReal() ? new RealVision() : new SimVision());
  }
    
  public VisionSubsystem(IVision implementation) {
    visionImplementation = implementation;
    tab = Shuffleboard.getTab("Vision");
    tab.addInteger("Target Id: ", () -> getTargetId());
    tab.addString("BotPose: ", () -> getBotpose().toString());
    tab.addBoolean("Has Speaker", () -> hasSpeaker());
    tab.addDouble("Horizontal Offset from Speaker", () -> getHorizontalSpeakerOffset().getDegrees());
    tab.addDouble("Vertical Offset from speaker", () -> getVerticalSpeakerOffset().getDegrees());
    tab.addDouble("Displacement from speaker", () -> getDistanceToSpeaker());
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

  public double getDistanceToSpeaker() {
    return visionImplementation.getHorizontalTranslation();
  }

  public LEDMode getLEDMode(){
    return visionImplementation.getLEDMode();
  }

  public void setLEDMode(LEDMode mode){
    visionImplementation.setLEDMode(mode);
  }
}
