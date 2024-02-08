package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  public static enum LEDMode {
    OFF,
    BLINK,
    ON;
  }

  private final IVision visionImplementation;
  private final ShuffleboardTab tab;
    
  public VisionSubsystem(IVision implementation){
    visionImplementation = implementation;
    tab = Shuffleboard.getTab("Vision");
    tab.addBoolean("Found target: ", () -> hasTarget());
    tab.addInteger("Target Id: ", () -> getTargetId());
    tab.addString("BotPose: ", () -> getBotpose().toString());
  }
  
  public boolean hasTarget(){
    return visionImplementation.hasTarget();
  }

  public Pose2d getBotpose(){
    return visionImplementation.getBotpose();
  }

  public int getTargetId(){
    return visionImplementation.getTargetId();
  }
  
  public LEDMode getLEDMode(){
    return visionImplementation.getLEDMode();
  }

  public void setLEDMode(LEDMode mode){
    visionImplementation.setLEDMode(mode);
  }

}
