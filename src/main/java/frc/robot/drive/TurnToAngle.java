package frc.robot.drive;

import edu.wpi.first.wpilibj2.command.Command;

public class TurnToAngle extends Command {
    
    private final DriveSubsystem drive;
    private final double targetAngle;

    public TurnToAngle(DriveSubsystem drive, double angleDeg) {
        this.drive = drive;
        this.targetAngle = angleDeg;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setTargetAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
