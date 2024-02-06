package frc.robot.notification;

public enum LEDStateEnum {
  RED(0.61, "Disabled"), 
  YELLOW(0.69, "Updating"), 
  GREEN(0.77, "Enabled");
  
  private final double spark;
  private final String message;

  private LEDStateEnum(double sparkValue, String statusMessage){
    this.spark = sparkValue; 
    this.message = statusMessage;
  } 

  public double getSparkValue(){
    return this.spark;
  }

  public String getStatusMessage(){
    return this.message;
  }

}
