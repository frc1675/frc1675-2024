package frc.robot.util;

import frc.robot.Constants;
import frc.robot.arm.Arm;
import frc.robot.shooter.ShooterSubsystem;

public class RobotContext {
    
    private final Arm arm;
    private final ShooterSubsystem shooter;

    private boolean intakeEnabled = true;

    public RobotContext(Arm arm, ShooterSubsystem shooter) {
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

    public double getDriveSpeedScale() {
        return arm.getAngle() <= Constants.Arm.HIGH_SCORE_POSITION ? Constants.Drive.SLOW_DRIVE_SCALE : 1;
    }

}
