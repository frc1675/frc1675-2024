package frc.robot.arm;

public class TestArmIO implements IArmIO {

  private double armMeasurement;
  private double motorPower;
  private boolean homeSwitch;

  public TestArmIO(){}

  public double getMotorSpeed(){
    return motorPower;
  }

  public void setMotorPower(double powerLevel){
    motorPower = powerLevel;
  }

  public double getMeasurement(){
    return armMeasurement;
  }

  public boolean atFrontLimit(){
    return homeSwitch;
  }

  public void setHomeSwitch(boolean state){
    homeSwitch = state;
  }

  public void setAngleMeasurement(double value){
    armMeasurement = value;
  }
  
  public void periodic(){}


}
