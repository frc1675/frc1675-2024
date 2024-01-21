package frc.robot.util.testing;

public class SimSparkMax implements TestableSparkMax {

    private double speed = 0;

    @Override
    public void setSpeed(double s) {
        speed = s;
    }

    @Override
    public double getSpeed() {
        return speed;
    }

    @Override
    public void close() {
        speed = 0;
    }
    
}
