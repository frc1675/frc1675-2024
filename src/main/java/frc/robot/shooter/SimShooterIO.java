package frc.robot.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SimShooterIO implements IShooterIO {
    private double shooterVoltage;
    private final FlywheelSim shooterMotorSim = new FlywheelSim(DCMotor.getNEO(1), Constants.Shooter.GEARING, Constants.Shooter.MOI);

    public SimShooterIO() {}

    @Override
    public void setIndexerOutput(double power) {
        
    }

    @Override
    public void setShooterOutput(double power) {
        shooterVoltage = power;
    }

    @Override
    public boolean isIndexerLoaded() {
        return true;
    }

    @Override
    public double[] getShooterSpeeds() {
        return new double[] {shooterMotorSim.getAngularVelocityRPM(), shooterMotorSim.getAngularVelocityRPM()};
    }

    @Override
    public double[] getIndexerSpeeds() {
        return new double[] {10000, 560000};
    }

    @Override
    public void periodic() {
        shooterMotorSim.setInputVoltage(shooterVoltage);
        shooterMotorSim.update(0.020);
    }
    
}
