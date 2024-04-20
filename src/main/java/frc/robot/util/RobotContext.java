package frc.robot.util;

import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.shooter.ShooterSubsystem;

public class RobotContext {

    private final ArmSubsystem arm;
    private final ShooterSubsystem shooter;

    private boolean intakeEnabled = true;

    public RobotContext(ArmSubsystem arm, ShooterSubsystem shooter) {
        this.arm = arm;
        this.shooter = shooter;
    }

    public void setIntakeEnabledOverride(boolean value) {
        intakeEnabled = value;
    }

    public boolean hasNote() {
        return shooter.isIndexerLoaded();
    }

    public boolean getIntakeEnabledOverride() {
        return intakeEnabled;
    }

    public boolean getReadyToIntake() {
        return arm.isAtHomePostion() && intakeEnabled;
    }

    public boolean shouldSlowShoot() {
        return arm.isAtAmpPosition();
    }

    public double[] getShooterSpeed() {
        if (arm.isAtAmpPosition()) {
            return new double[] {Constants.Shooter.AMP_SHOOT_SPEED, Constants.Shooter.AMP_SHOOT_SPEED};
        } else if (arm.isAtHomePostion()) {
            return new double[] {Constants.Shooter.SHOOT_SPEED, Constants.Shooter.SHOOT_SPEED * 0.9};
        } else {
            return new double[] {Constants.Shooter.LONG_SHOT_SPEED, Constants.Shooter.LONG_SHOT_SPEED};
        }
    }

    public double getDriveSpeedScale() {
        return arm.getAngle() <= Constants.Arm.HIGH_SCORE_POSITION ? Constants.Drive.SLOW_DRIVE_SCALE : 1;
    }
}
