package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import au.grapplerobotics.LaserCan;
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

    //private LaserCan laserCAN;

    public RealShooterIO() {
        shooterMotorOne = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_ONE, CANSparkMax.MotorType.kBrushless);
        shooterMotorTwo = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_TWO, CANSparkMax.MotorType.kBrushless);
        indexerMotorOne = new CANSparkMax(Constants.Shooter.INDEXER_MOTOR_ONE, CANSparkMax.MotorType.kBrushless);
        indexerMotorTwo = new CANSparkMax(Constants.Shooter.INDEXER_MOTOR_TWO, CANSparkMax.MotorType.kBrushless);

        shooterMotorOne.setInverted(true);
        shooterMotorTwo.setInverted(true);

        shooterMotorOneEncoder = shooterMotorOne.getEncoder();
        shooterMotorTwoEncoder = shooterMotorTwo.getEncoder();
        indexerMotorOneEncoder = indexerMotorOne.getEncoder();
        indexerMotorTwoEncoder = indexerMotorTwo.getEncoder();

        //laserCAN = new LaserCan(Constants.Shooter.LASER_CAN);
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
        return false;
        //LaserCan.Measurement measurement = laserCAN.getMeasurement();
        //return measurement.distance_mm < Constants.Shooter.INDEXER_NOTE_DETECTION_RANGE;
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
