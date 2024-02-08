package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.vision.VisionSubsystem.LEDMode;

public class RealVision implements IVision {

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ups");
  private final NetworkTableEntry aprilTagID = table.getEntry("tid");
  private final NetworkTableEntry isValidTarget = table.getEntry("tv"); 
  private final NetworkTableEntry ledMode = table.getEntry("LEDMode");
  private LEDMode currentMode;
  private NetworkTableEntry botpose = null;

  public RealVision(){
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        botpose = table.getEntry("botpose_wpired");
      }else{
        botpose = table.getEntry("botpose_wpiblue");
      }
    }
    ledMode.setNumber(3);
  }

  public Pose2d getBotpose(){
    double[] botposeArray = botpose.getDoubleArray(new double[6]);
    return new Pose2d(botposeArray[0], botposeArray[1], Rotation2d.fromDegrees(botposeArray[5]));
  }

  public boolean hasTarget(){
    return isValidTarget.getDouble(0) == 1;
  }

  public int getTargetId(){
    return (int) aprilTagID.getInteger(0);
  }

  public void setLEDMode(LEDMode ledState){
    switch (ledState) {
      case OFF:
        ledMode.setNumber(3);
        currentMode = LEDMode.OFF;
        break;
      case BLINK:
        ledMode.setNumber(2);
        currentMode = LEDMode.BLINK;
        break;
      case ON:
        ledMode.setNumber(1); 
        currentMode = LEDMode.ON;
        break;
      default:
        DataLogManager.log("I have no clue how this happened?");
        break;
    }
  } 

  public LEDMode getLEDMode(){
    return currentMode;
  }


}
