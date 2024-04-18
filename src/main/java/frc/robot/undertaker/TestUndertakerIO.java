package frc.robot.undertaker;

public class TestUndertakerIO implements IUndertaker {

  private boolean[] mockMotorStatus = {false, false};
  private double desiredSpeed;
  private boolean hasRun = false;

  public TestUndertakerIO() {}

  @Override
  public double getDesiredSpeed() {
    return desiredSpeed;
  }

  @Override
  public double[] getMotorsOutput() {
    double[] motorsOutput = new double[2];
    motorsOutput[0] = desiredSpeed * 12;
    motorsOutput[1] = desiredSpeed * 12;
    return motorsOutput;
  }

  @Override
  public void run(double speed) {
    if (mockMotorStatus[0] && mockMotorStatus[1]) {
      this.desiredSpeed = speed;
      hasRun = true;
    }
  }

  @Override
  public boolean[] isAlive() {
    return mockMotorStatus;
  }

  public void setIsAlive(boolean[] status) {
    mockMotorStatus = status;
  }

  public boolean getHasRun() {
    return hasRun;
  }

  public void resetHasRun() {
    hasRun = false;
  }
}
