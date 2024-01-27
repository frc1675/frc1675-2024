package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;

import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax indexerMotorOne = new CANSparkMax(Constants.Shooter.INDEXER_MOTOR_ONE, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax indexerMotorTwo = new CANSparkMax(Constants.Shooter.INDEXER_MOTOR_TWO, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax shooterMotorOne = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_ONE, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax shooterMotorTwo = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_TWO, CANSparkMax.MotorType.kBrushless);
    private RelativeEncoder shooterMotorOneEncoder = shooterMotorOne.getEncoder();
    private RelativeEncoder shooterMotorTwoEncoder = shooterMotorTwo.getEncoder();

    private LaserCan laserCAN = new LaserCan(Constants.Shooter.LASER_CAN);
    
    private double targetShooterSpeed = 0;

    public ShooterSubsystem() {
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(indexerMotorOne, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(indexerMotorTwo, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(shooterMotorOne, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(shooterMotorTwo, DCMotor.getNEO(1));
        }
    }

    public boolean isIndexerLoaded() {
        LaserCan.Measurement measurement = laserCAN.getMeasurement();
        return measurement.distance_mm < Constants.Shooter.INDEXER_NOTE_DETECTION_RANGE;
    }

    public boolean isShooterReady() {
        double shooterOneSpeedOffset = shooterMotorOneEncoder.getVelocity() - targetShooterSpeed;
        double shooterTwoSpeedOffset = shooterMotorTwoEncoder.getVelocity() - targetShooterSpeed;
        return Math.abs(shooterOneSpeedOffset) <= Constants.Shooter.TARGET_SPEED_ERROR_MARGIN && Math.abs(shooterTwoSpeedOffset) <= Constants.Shooter.TARGET_SPEED_ERROR_MARGIN;
    }

    public void setTargetShooterSpeed(double targetSpeed) {
        shooterMotorOne.setVoltage(targetSpeed * 12); // TODO: use PID loop to update voltage
        shooterMotorTwo.setVoltage(targetSpeed * 12);
        targetShooterSpeed = targetSpeed;
    }

    public void setIndexerSpeed(double speed) {
        indexerMotorOne.setVoltage(speed * 12); // TODO: convert m/s to volts (currently ratio to volts)
        indexerMotorTwo.setVoltage(speed * 12);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}