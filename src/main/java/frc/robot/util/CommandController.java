package frc.robot.util;

import frc.robot.Constants;
import frc.robot.arm.Arm;

public class CommandController {
    
    private final Arm arm;

    private boolean intakeEnabled = true;

    public CommandController(Arm arm) {
        this.arm = arm;
    }

    public void setIntakeEnabledOverride(boolean value) {
        intakeEnabled = value;
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
