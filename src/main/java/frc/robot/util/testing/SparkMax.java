package frc.robot.util.testing;

import com.revrobotics.CANSparkMax;

public class SparkMax implements TestableSparkMax {

    private CANSparkMax motor;

    public SparkMax(CANSparkMax motor) {
        this.motor = motor;
    }

    @Override
    public void setSpeed(double s) {
        motor.set(s);
    }

    @Override
    public double getSpeed() {
        return motor.get();
    }

    @Override
    public void close() {
        motor.close();
    }
    
}
