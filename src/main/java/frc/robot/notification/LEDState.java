package frc.robot.notification;

public enum LEDState {
  UNDERTAKER_DISABLED(0.61, "Undertaker Disabled", "ff0000", -1), //RED
  SPINNING_UP(0.69, "Motors spinning up", "ffff00", -1), //YELLOW
  HAS_NOTE(0.77, "Has note", "00ff00", 2), //GREEN
  NOTHING(-0.07 ,"Nothing", "000000", -1); //GOLD
  
  private final double spark;
  private final String message;
  private final String hexCode;
  private final double timeout;

  private double timeAdded = -1;

  private LEDState(double sparkValue, String statusMessage, String hexCode, double timeout){
    this.spark = sparkValue; 
    this.message = statusMessage;
    this.hexCode = hexCode;
    this.timeout = timeout;
  }

  public void setTimeAdded(double time) {
    timeAdded = time;
  }

  public boolean isExpired(double time) {
    if (timeAdded < 0 || timeout < 0) {
      return false;
    }

    return (timeAdded - time) >= timeout;
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

  public double getTimeout() {
    return timeout;
  }

}
