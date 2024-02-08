package frc.robot.notification;

public class SimLedIO implements ILedIO{

  private double ledSparkValue = 0;
  
  public SimLedIO(){}

  @Override
  public void setSpark(double sparkValue){
    ledSparkValue = sparkValue;
  }

  public double getLedSparkValue(){
    return ledSparkValue;
  }

  @Override
  public boolean getIsAlive(){
    return true;
  }

}
