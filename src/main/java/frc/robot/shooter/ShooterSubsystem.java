package frc.robot.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
    private IShooterIO shooterIO;
    private double targetTopShooterSpeed = 0;
    private double targetBottomShooterSpeed = 0;
    private double targetIndexerSpeed = 0;

    private double topOutput = 0;
    private double bottomOutput = 0;

    private PIDController topPidController = new PIDController(
            Constants.Shooter.SHOOTER_PID_P, Constants.Shooter.SHOOTER_PID_I, Constants.Shooter.SHOOTER_PID_D);
    private SimpleMotorFeedforward topFeedForward =
            new SimpleMotorFeedforward(Constants.Shooter.SHOOTER_FF_S, Constants.Shooter.SHOOTER_FF_V);

    private PIDController bottomPidController = new PIDController(
            Constants.Shooter.SHOOTER_PID_P, Constants.Shooter.SHOOTER_PID_I, Constants.Shooter.SHOOTER_PID_D);
    private SimpleMotorFeedforward bottomFeedForward =
            new SimpleMotorFeedforward(Constants.Shooter.SHOOTER_FF_S, Constants.Shooter.SHOOTER_FF_V);

    public static ShooterSubsystem create() {
        return new ShooterSubsystem(Robot.isReal() ? new RealShooterIO() : new SimShooterIO());
    }

    public ShooterSubsystem(IShooterIO shooterIO) {
        this.shooterIO = shooterIO;
        ShuffleboardInit();
    }

    private void ShuffleboardInit() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Shooter.SHUFFLEBOARD_TAB);
        tab.addDouble("Target Top Shooter Speed", () -> targetTopShooterSpeed)
                .withPosition(0, 0)
                .withSize(2, 1);
        tab.addDouble("Target Bottom Shooter Speed", () -> targetBottomShooterSpeed)
                .withPosition(0, 1)
                .withSize(2, 1);
        tab.addDouble("Shooter 1 Speed", () -> shooterIO.getShooterSpeeds()[0]).withPosition(2, 0);
        tab.addDouble("Shooter 2 Speed", () -> shooterIO.getShooterSpeeds()[1]).withPosition(3, 0);
        tab.addDouble(
                        "Shooter Speed Diff.",
                        () -> shooterIO.getShooterSpeeds()[0] - shooterIO.getShooterSpeeds()[1])
                .withPosition(4, 0)
                .withSize(2, 1);

        tab.addBoolean("Shooter Ready?", () -> isShooterReady())
                .withPosition(3, 2)
                .withSize(3, 3);
        tab.addBoolean("Has Note?", () -> isIndexerLoaded()).withPosition(6, 1).withSize(4, 4);

        tab.addDouble("Target Indexer Speed", () -> targetIndexerSpeed)
                .withPosition(0, 2)
                .withSize(2, 1);
        tab.addDouble("Indexer 1 Speed", () -> shooterIO.getIndexerSpeeds()[0]).withPosition(2, 1);
        tab.addDouble("Indexer 2 Speed", () -> shooterIO.getIndexerSpeeds()[1]).withPosition(3, 1);
        tab.addDouble(
                        "Indexer Speed Diff.",
                        () -> shooterIO.getIndexerSpeeds()[0] - shooterIO.getIndexerSpeeds()[1])
                .withPosition(4, 1)
                .withSize(2, 1);
        tab.addDouble("LaserCAN Measurement", () -> shooterIO.getMeasurement())
                .withPosition(0, 3)
                .withSize(2, 1);
    }

    public boolean isIndexerLoaded() {
        return shooterIO.isIndexerLoaded();
    }

    public boolean isShooterReady() {
        double[] speeds = shooterIO.getShooterSpeeds();
        return Math.abs(targetTopShooterSpeed - speeds[0]) < Constants.Shooter.TARGET_SPEED_ERROR_MARGIN
                && Math.abs(targetBottomShooterSpeed - speeds[1]) < Constants.Shooter.TARGET_SPEED_ERROR_MARGIN;
    }

    public void setTargetShooterSpeeds(double targetTopSpeed, double targetBottomSpeed) {
        targetTopShooterSpeed = targetTopSpeed;
        targetBottomShooterSpeed = targetBottomSpeed;
    }

    public void setIndexerSpeed(double targetSpeed) {
        targetIndexerSpeed = targetSpeed;
        shooterIO.setIndexerOutput(targetIndexerSpeed);
    }

    public Command testMotor() {
        return new InstantCommand(() -> shooterIO.setShooterOutput(0.1, 0))
                .andThen(new WaitCommand(2))
                .andThen(() -> shooterIO.setShooterOutput(0, 0));
    }

    @Override
    public void periodic() {

        topOutput = topPidController.calculate(shooterIO.getShooterSpeeds()[0], targetTopShooterSpeed)
                + topFeedForward.calculate(targetTopShooterSpeed);
        bottomOutput = bottomPidController.calculate(shooterIO.getShooterSpeeds()[1], targetBottomShooterSpeed)
                + bottomFeedForward.calculate(targetBottomShooterSpeed);
        shooterIO.setShooterOutput(topOutput, bottomOutput);

        shooterIO.periodic();
    }
}
