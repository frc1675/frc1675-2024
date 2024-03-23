package frc.robot.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TurnToAngle extends Command {
    
    private final DriveSubsystem drive;
    private final DoubleSupplier targetAngle;

    public TurnToAngle(DriveSubsystem drive, DoubleSupplier angleDeg) {
        this.drive = drive;
        this.targetAngle = angleDeg;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setTargetAngle(targetAngle.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
