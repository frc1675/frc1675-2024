package frc.robot.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private IShooterIO shooterIO;
    private double targetShooterSpeed = 0;
    private double targetIndexerSpeed = 0;
    private PIDController shooterPidController = new PIDController(Constants.Shooter.SHOOTER_PID_P, Constants.Shooter.SHOOTER_PID_I, Constants.Shooter.SHOOTER_PID_D);
    private SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(Constants.Shooter.SHOOTER_FF_S, Constants.Shooter.SHOOTER_FF_V);

    public ShooterSubsystem(IShooterIO shooterIO) {
        this.shooterIO = shooterIO;
        ShuffleboardInit();
    }

    private void ShuffleboardInit() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Shooter.SHUFFLEBOARD_TAB);
        tab.addDouble("Target Shooter Speed", () -> targetShooterSpeed).withPosition(0, 0).withSize(2, 1);
        tab.addDouble("Shooter 1 Speed", () -> shooterIO.getShooterSpeeds()[0]).withPosition(2, 0);
        tab.addDouble("Shooter 2 Speed", () -> shooterIO.getShooterSpeeds()[1]).withPosition(3, 0);
        tab.addDouble("Shooter Speed Dif.", () -> shooterIO.getShooterSpeeds()[0] - shooterIO.getShooterSpeeds()[1]).withPosition(4, 0).withSize(2, 1);

        tab.addBoolean("Shooter Ready?", () -> isShooterReady()).withPosition(6, 0);
        tab.addBoolean("Has Note?", () -> isIndexerLoaded()).withPosition(6, 1);

        tab.addDouble("Target Indexer Speed", () -> targetIndexerSpeed).withPosition(0, 1).withSize(2, 1);
        tab.addDouble("Indexer 1 Speed", () -> shooterIO.getIndexerSpeeds()[0]).withPosition(2, 1);
        tab.addDouble("Indexer 2 Speed", () -> shooterIO.getIndexerSpeeds()[1]).withPosition(3, 1);
        tab.addDouble("Indexer Speed Dif.", () -> shooterIO.getIndexerSpeeds()[0] - shooterIO.getIndexerSpeeds()[1]).withPosition(4, 1).withSize(2, 1);
        tab.addDouble("sensor mesurment", () -> shooterIO.getMeasurement());
        //tab.add(pidController).withWidget(BuiltInWidgets.kPIDController).withPosition(5, 1);
        //tab.addDouble("PID Input", () -> pidOutput).withPosition(4, 1);
    }

    public boolean isIndexerLoaded() {
        return shooterIO.isIndexerLoaded();
    }

    public boolean isShooterReady() {
        double[] speeds = shooterIO.getShooterSpeeds();
        return Math.abs(targetShooterSpeed - speeds[0]) < Constants.Shooter.TARGET_SPEED_ERROR_MARGIN && Math.abs(targetShooterSpeed - speeds[1]) < Constants.Shooter.TARGET_SPEED_ERROR_MARGIN;
    }

    public void setTargetShooterSpeed(double targetSpeed) {
        //pidController.setSetpoint(targetSpeed);
        targetShooterSpeed = targetSpeed;
    }

    public void setIndexerSpeed(double targetSpeed) {
        targetIndexerSpeed = targetSpeed;
        shooterIO.setIndexerOutput(targetIndexerSpeed);
    }

    @Override
    public void periodic() {
        /*shooterPidOutput = shooterPidController.calculate(shooterIO.getShooterSpeeds()[0], targetShooterSpeed);
        shooterFfOutput = shooterFeedForward.calculate(targetShooterSpeed);
        shooterIO.setShooterOutput(shooterPidOutput + shooterFfOutput);*/
  /*       
        indexerPidOutput = indexerPidController.calculate(shooterIO.getIndexerSpeeds()[0], targetIndexerSpeed);
        indexerFfOutput = indexerFeedForward.calculate(targetIndexerSpeed);
        shooterIO.setIndexerOutput(indexerPidOutput + indexerFfOutput);
*/
        shooterIO.setShooterOutput(targetShooterSpeed);

        shooterIO.periodic();
    }

    @Override
    public void simulationPeriodic() {}
}
