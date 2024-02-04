package frc.robot.notification;

public enum LEDStateEnum {
  COLOR_RED_SPARK(0.61, "Disabled"), 
  COLOR_YELLOW_SPARK(0.69, "Updating"), 
  COLOR_GREEN_SPARK(0.77, "Enabled");
  
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
