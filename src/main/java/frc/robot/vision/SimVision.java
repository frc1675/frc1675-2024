package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.VisionSubsystem.LEDMode;

public class SimVision implements IVision {

  private Pose2d botPose;
  private boolean hasTarget = false;
  private int targetId = 0;
  private LEDMode currentMode;
  
  public SimVision(){}

  @Override
  public Pose2d getBotpose(){
    return botPose;
  }

  public void setBotPose(Pose2d targetPose){
    this.botPose = targetPose;
  } 

  @Override
  public boolean hasTarget(){
    return hasTarget;
  }

  public void setHasTarget(boolean value){
    hasTarget = value;
  }

  @Override
  public int getTargetId(){
    return targetId;
  }

  public void setTargetId(int id){
    targetId = id;
  }

  @Override
  public LEDMode getLEDMode(){
    return currentMode;
  }

  @Override
  public void setLEDMode(LEDMode mode){
    currentMode = mode;
  }

}
