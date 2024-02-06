package frc.robot.shooter;

public interface IShooterIO {

    public void setIndexerOutput(double power);

    public void setShooterOutput(double power);

    public boolean isIndexerLoaded();

    public double[] getShooterSpeeds();

    public double[] getIndexerSpeeds();

    public void periodic();
}