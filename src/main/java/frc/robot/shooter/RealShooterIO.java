package frc.robot.shooter;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class RealShooterIO implements IShooterIO {

    private SparkMax shooterMotorTop;
    private SparkMax shooterMotorBottom;
    private SparkMax indexerMotorTop;
    private SparkMax indexerMotorBottom;

    private RelativeEncoder shooterMotorTopEncoder;
    private RelativeEncoder shooterMotorBottomEncoder;
    private RelativeEncoder indexerMotorTopEncoder;
    private RelativeEncoder indexerMotorBottomEncoder;

    private LaserCan laserCAN;

    public RealShooterIO() {
        shooterMotorTop = new SparkMax(Constants.Shooter.SHOOTER_MOTOR_TOP, MotorType.kBrushless);
        shooterMotorBottom = new SparkMax(Constants.Shooter.SHOOTER_MOTOR_BOTTOM, MotorType.kBrushless);
        indexerMotorTop = new SparkMax(Constants.Shooter.INDEXER_MOTOR_TOP, MotorType.kBrushless);
        indexerMotorBottom = new SparkMax(Constants.Shooter.INDEXER_MOTOR_BOTTOM, MotorType.kBrushless);

        // indexerMotorTop.setIdleMode(IdleMode.kBrake);
        // indexerMotorBottom.setIdleMode(IdleMode.kBrake);

        shooterMotorTop.setInverted(true);
        shooterMotorBottom.setInverted(true);

        indexerMotorTop.setInverted(false);
        indexerMotorBottom.setInverted(false);

        shooterMotorTopEncoder = shooterMotorTop.getEncoder();
        shooterMotorBottomEncoder = shooterMotorBottom.getEncoder();
        indexerMotorTopEncoder = indexerMotorTop.getEncoder();
        indexerMotorBottomEncoder = indexerMotorBottom.getEncoder();

        laserCAN = new LaserCan(Constants.Shooter.LASER_CAN);

        try {
            laserCAN.setRangingMode(RangingMode.SHORT);
            laserCAN.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void setIndexerOutput(double power) {
        indexerMotorTop.setVoltage(Math.min(1, Math.max(power, -1)) * 12);
        indexerMotorBottom.setVoltage(Math.min(1, Math.max(power, -1)) * 12);
    }

    @Override
    public void setShooterOutput(double topPower, double bottomPower) {
        shooterMotorTop.setVoltage(topPower * 12);
        shooterMotorBottom.setVoltage(bottomPower * 12);
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
        return new double[] {shooterMotorTopEncoder.getVelocity(), shooterMotorBottomEncoder.getVelocity()};
    }

    @Override
    public double[] getIndexerSpeeds() {
        return new double[] {indexerMotorTopEncoder.getVelocity(), indexerMotorBottomEncoder.getVelocity()};
    }

    @Override
    public void periodic() {}
}
