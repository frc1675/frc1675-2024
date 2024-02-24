package frc.robot.notification;

public enum LEDStateEnum {
  RED(0.61, "Disabled", "ff0000"), 
  YELLOW(0.69, "Updating", "ffff00"), 
  GREEN(0.77, "Enabled", "00ff00");
  
  private final double spark;
  private final String message;
  private final String hexCode;

  private LEDStateEnum(double sparkValue, String statusMessage, String hexCode){
    this.spark = sparkValue; 
    this.message = statusMessage;
    this.hexCode = hexCode;
  } 

  public double getSparkValue(){
    return this.spark;
  }

  public String getStatusMessage(){
    return this.message;
  }

  public String getHexCode(){
    return this.hexCode;
  }

}
