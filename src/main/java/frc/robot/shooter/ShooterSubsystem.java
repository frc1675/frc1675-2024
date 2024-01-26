package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax indexerMotor = new CANSparkMax(Constants.Shooter.INDEXER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax shooterMotorOne = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_ONE_ID, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax shooterMotorTwo = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_TWO_ID, CANSparkMax.MotorType.kBrushless);
    private RelativeEncoder shooterMotorOneEncoder = shooterMotorOne.getEncoder();
    private RelativeEncoder shooterMotorTwoEncoder = shooterMotorTwo.getEncoder();

    private LaserCan laserCAN = new LaserCan(Constants.Shooter.LASER_CAN_ID);
    
    private double targetShooterSpeed = 0;

    public boolean isIndexerLoaded() {
        LaserCan.Measurement measurement = laserCAN.getMeasurement(); // CREDIT NOAH WEISHAN
        return measurement.distance_mm < Constants.Shooter.INDEXER_NOTE_DETECTION_RANGE;
    }

    public boolean isShooterReady() {
        double shooterOneSpeedOffset = shooterMotorOneEncoder.getVelocity() - targetShooterSpeed;
        double shooterTwoSpeedOffset = shooterMotorTwoEncoder.getVelocity() - targetShooterSpeed;
        return Math.abs(shooterOneSpeedOffset) < Constants.Shooter.TARGET_SPEED_ERROR_MARGIN && Math.abs(shooterTwoSpeedOffset) < Constants.Shooter.TARGET_SPEED_ERROR_MARGIN;
    }

    public void setTargetShooterSpeed(double targetSpeed) {
        shooterMotorOne.set(targetSpeed);
        shooterMotorTwo.set(targetSpeed);
        targetShooterSpeed = targetSpeed; // TODO: NEEDS TO CONVERT RATIO TO RPM
    }

    public void setTargetIndexerSpeed(double targetSpeed) {
        indexerMotor.set(targetSpeed);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}