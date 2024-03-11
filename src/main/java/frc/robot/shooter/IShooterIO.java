package frc.robot.shooter;

public interface IShooterIO {
  public void setIndexerOutput(double power);

  public void setShooterOutput(double topPower, double bottomPower);

  public boolean isIndexerLoaded();

  public double[] getShooterSpeeds();

  public double[] getIndexerSpeeds();

  public double getMeasurement();

  public void periodic();
}
