package frc.robot.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private IShooterIO shooterIO;
    private double targetShooterSpeed = 0;
    private double targetIndexerSpeed = 0;
    
    private PIDController topPidController = new PIDController(Constants.Shooter.SHOOTER_PID_P, Constants.Shooter.SHOOTER_PID_I, Constants.Shooter.SHOOTER_PID_D);
    private SimpleMotorFeedforward topFeedForward = new SimpleMotorFeedforward(Constants.Shooter.SHOOTER_FF_S, Constants.Shooter.SHOOTER_FF_V);

    private PIDController bottomPidController = new PIDController(Constants.Shooter.SHOOTER_PID_P, Constants.Shooter.SHOOTER_PID_I, Constants.Shooter.SHOOTER_PID_D);
    private SimpleMotorFeedforward bottomFeedForward = new SimpleMotorFeedforward(Constants.Shooter.SHOOTER_FF_S, Constants.Shooter.SHOOTER_FF_V);

    public ShooterSubsystem(IShooterIO shooterIO) {
        this.shooterIO = shooterIO;
        ShuffleboardInit();

        topPidController.setTolerance(Constants.Shooter.TARGET_SPEED_ERROR_MARGIN);
        bottomPidController.setTolerance(Constants.Shooter.TARGET_SPEED_ERROR_MARGIN);
    }

    private void ShuffleboardInit() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Shooter.SHUFFLEBOARD_TAB);
        tab.addDouble("Target Shooter Speed", () -> targetShooterSpeed).withPosition(0, 0).withSize(2, 1);
        tab.addDouble("Shooter 1 Speed", () -> shooterIO.getShooterSpeeds()[0]).withPosition(2, 0);
        tab.addDouble("Shooter 2 Speed", () -> shooterIO.getShooterSpeeds()[1]).withPosition(3, 0);
        tab.addDouble("Shooter Speed Diff.", () -> shooterIO.getShooterSpeeds()[0] - shooterIO.getShooterSpeeds()[1]).withPosition(4, 0).withSize(2, 1);

        tab.addBoolean("Shooter Ready?", () -> isShooterReady()).withPosition(6, 0);
        tab.addBoolean("Has Note?", () -> isIndexerLoaded()).withPosition(6, 1);

        tab.addDouble("Target Indexer Speed", () -> targetIndexerSpeed).withPosition(0, 1).withSize(2, 1);
        tab.addDouble("Indexer 1 Speed", () -> shooterIO.getIndexerSpeeds()[0]).withPosition(2, 1);
        tab.addDouble("Indexer 2 Speed", () -> shooterIO.getIndexerSpeeds()[1]).withPosition(3, 1);
        tab.addDouble("Indexer Speed Diff.", () -> shooterIO.getIndexerSpeeds()[0] - shooterIO.getIndexerSpeeds()[1]).withPosition(4, 1).withSize(2, 1);
        tab.addDouble("LaserCAN Measurement", () -> shooterIO.getMeasurement());
    }

    public boolean isIndexerLoaded() {
        return shooterIO.isIndexerLoaded();
    }

    public boolean isShooterReady() {
        return topPidController.atSetpoint() && bottomPidController.atSetpoint();
    }

    public void setTargetShooterSpeed(double targetSpeed) {
        targetShooterSpeed = targetSpeed;
    }

    public void setIndexerSpeed(double targetSpeed) {
        targetIndexerSpeed = targetSpeed;
        shooterIO.setIndexerOutput(targetIndexerSpeed);
    }

    @Override
    public void periodic() {
        shooterIO.setShooterOutput(
            topPidController.calculate(shooterIO.getShooterSpeeds()[0], targetShooterSpeed)
            + topFeedForward.calculate(targetShooterSpeed),
            bottomPidController.calculate(shooterIO.getShooterSpeeds()[1], targetShooterSpeed * 0.9)
            + bottomFeedForward.calculate(targetShooterSpeed * 0.9)
        );

        shooterIO.periodic();
    }

}
