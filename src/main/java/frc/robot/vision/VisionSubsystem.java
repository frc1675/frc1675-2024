package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    tab.addBoolean("Found target: ", () -> hasTarget());
    tab.addInteger("Target Id: ", () -> getTargetId());
    tab.addString("BotPose: ", () -> getBotpose().toString());
    tab.addDouble("Distance to Speaker", () -> getHorizontalSpeakerDistance());
    tab.addDouble("Offset from speaker", () -> getTargetOffset().getDegrees());
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

  public double getHorizontalSpeakerDistance() {
    return visionImplementation.getHorizontalSpeakerDistance(visionImplementation.getTargetTranslation());
  }

  public Rotation2d getTargetOffset() {
    return visionImplementation.getTargetOffset();
  }
  
  public LEDMode getLEDMode(){
    return visionImplementation.getLEDMode();
  }

  public void setLEDMode(LEDMode mode){
    visionImplementation.setLEDMode(mode);
  }
}
