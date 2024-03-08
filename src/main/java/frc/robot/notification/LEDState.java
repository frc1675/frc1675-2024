package frc.robot.notification;

public enum LEDState {
  UNDERTAKER_DISABLED(0.61, "Undertaker Disabled", "#ff0000", -1), //RED
  SPINNING_UP(0.69, "Motors spinning up", "#ffff00", -1), //YELLOW
  HAS_NOTE(0.77, "Has note", "#00ff00", 2), //GREEN
  NOTHING(-0.07 ,"Nothing", "#ffffff", -1); //WHITE
  
  private final double spark;
  private final String message;
  private final String hexCode;
  private final double timeout;

  private double timeAdded = -1;
  private boolean removeImmediately = false;

  private LEDState(double sparkValue, String statusMessage, String hexCode, double timeout){
    this.spark = sparkValue; 
    this.message = statusMessage;
    this.hexCode = hexCode;
    this.timeout = timeout;
  }

  public void requestRemoval() {
    removeImmediately = true;
  }

  public void setTimeAdded(double time) {
    timeAdded = time;
  }

  public boolean isExpired(double time) {

    if (removeImmediately) {
      removeImmediately = false;
      return true;
    }

    if (timeAdded < 0 || timeout < 0) {
      return false;
    }

    return (time - timeAdded) >= timeout;
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
