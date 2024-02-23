package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import frc.robot.Constants;

public class RealShooterIO implements IShooterIO {
   
    private CANSparkMax shooterMotorOne;
    private CANSparkMax shooterMotorTwo;
    private CANSparkMax indexerMotorOne;
    private CANSparkMax indexerMotorTwo;

    private RelativeEncoder shooterMotorOneEncoder;
    private RelativeEncoder shooterMotorTwoEncoder;
    private RelativeEncoder indexerMotorOneEncoder;
    private RelativeEncoder indexerMotorTwoEncoder;

    private LaserCan laserCAN;

    public RealShooterIO() {
        shooterMotorOne = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_ONE, CANSparkMax.MotorType.kBrushless);
        shooterMotorTwo = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_TWO, CANSparkMax.MotorType.kBrushless);
        indexerMotorOne = new CANSparkMax(Constants.Shooter.INDEXER_MOTOR_ONE, CANSparkMax.MotorType.kBrushless);
        indexerMotorTwo = new CANSparkMax(Constants.Shooter.INDEXER_MOTOR_TWO, CANSparkMax.MotorType.kBrushless);

        indexerMotorOne.setIdleMode(IdleMode.kBrake);
        indexerMotorTwo.setIdleMode(IdleMode.kBrake);

        shooterMotorOne.setInverted(true);
        shooterMotorTwo.setInverted(true);

        indexerMotorOne.setInverted(false);
        indexerMotorTwo.setInverted(false);

        shooterMotorOneEncoder = shooterMotorOne.getEncoder();
        shooterMotorTwoEncoder = shooterMotorTwo.getEncoder();
        indexerMotorOneEncoder = indexerMotorOne.getEncoder();
        indexerMotorTwoEncoder = indexerMotorTwo.getEncoder();

        laserCAN = new LaserCan(Constants.Shooter.LASER_CAN);
        
        // configure the laserCAN sensor
        try {
            laserCAN.setRangingMode(RangingMode.SHORT);
            laserCAN.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
        } catch (ConfigurationFailedException e) {
        	e.printStackTrace();
        }
    }

    @Override
    public void setIndexerOutput(double power) {
        indexerMotorOne.setVoltage(Math.min(1, Math.max(power, -1)) * 12);
        indexerMotorTwo.setVoltage(Math.min(1, Math.max(power, -1)) * 12);
    }

    @Override
    public void setShooterOutput(double power) {
        shooterMotorOne.setVoltage(Math.min(1, Math.max(power, -1)) * 12);
        shooterMotorTwo.setVoltage(Math.min(1, Math.max(power, -1)) * 12);
    }

    @Override
    public boolean isIndexerLoaded() {
        LaserCan.Measurement measurement = laserCAN.getMeasurement();
        return measurement.distance_mm < Constants.Shooter.INDEXER_NOTE_DETECTION_RANGE;
    }

    public double getMeasurement() {
        return laserCAN.getMeasurement() == null ? 0 : laserCAN.getMeasurement().distance_mm;
    }

    @Override
    public double[] getShooterSpeeds() {
        return new double[] { shooterMotorOneEncoder.getVelocity(), shooterMotorTwoEncoder.getVelocity() };
    }

    @Override
    public double[] getIndexerSpeeds() {
        return new double[] { indexerMotorOneEncoder.getVelocity(), indexerMotorTwoEncoder.getVelocity() };
    }

    @Override
    public void periodic() {}
}
