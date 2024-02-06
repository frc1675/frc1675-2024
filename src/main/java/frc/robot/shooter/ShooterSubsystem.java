package frc.robot.shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private IShooterIO shooterIO = new RealShooterIO();
    private double targetShooterSpeed = 0;

    public ShooterSubsystem() {
        ShuffleboardInit();
    }

    private void ShuffleboardInit() {
        double[] shooterSpeeds = shooterIO.getShooterSpeeds();
        double[] indexerSpeeds = shooterIO.getIndexerSpeeds();

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Shooter.SHUFFLEBOARD_TAB);
        tab.addDouble("Target Shooter Speed", () -> targetShooterSpeed).withPosition(0, 0).withSize(2, 1);
        tab.addDouble("Shooter 1 Speed", () -> shooterSpeeds[0]).withPosition(2, 0);
        tab.addDouble("Shooter 2 Speed", () -> shooterSpeeds[1]).withPosition(3, 0);
        tab.addDouble("Shooter Speed Dif.", () -> shooterSpeeds[0] - shooterSpeeds[1]).withPosition(4, 0).withSize(2, 1);
        tab.addBoolean("Shooter Ready?", () -> isShooterReady()).withPosition(6, 0);
        tab.addBoolean("Has Note?", () -> isIndexerLoaded()).withPosition(6, 1);
        tab.addDouble("Indexer 1 Speed", () -> indexerSpeeds[0]).withPosition(0, 1);
        tab.addDouble("Indexer 2 Speed", () -> indexerSpeeds[1]).withPosition(1, 1);
        tab.addDouble("Indexer Speed Dif.", () -> indexerSpeeds[0] - indexerSpeeds[1]).withPosition(2, 1);
    }

    public boolean isIndexerLoaded() {
        return shooterIO.isIndexerLoaded();
    }

    public boolean isShooterReady() {
        double[] speeds = shooterIO.getShooterSpeeds();
        return Math.abs(targetShooterSpeed - speeds[0]) < Constants.Shooter.TARGET_SPEED_ERROR_MARGIN && Math.abs(targetShooterSpeed - speeds[1]) < Constants.Shooter.TARGET_SPEED_ERROR_MARGIN;
    }

    public void setTargetShooterSpeed(double targetSpeed) {
        targetShooterSpeed = targetSpeed;
        shooterIO.setShooterOutput(targetSpeed); // TODO: convert RPM to -1 to 1
    }

    public void setIndexerSpeed(double speed) {
        shooterIO.setIndexerOutput(speed); // TODO: convert RPM to -1 to 1
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
