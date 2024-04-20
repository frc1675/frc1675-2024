package frc.robot.undertaker;

public interface IUndertaker {

    public void run(double speed);

    public boolean[] isAlive();

    public double[] getMotorsOutput();

    public double getDesiredSpeed();
}
