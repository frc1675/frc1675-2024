package frc.robot.auto.simple;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.drive.DriveSubsystem;

public class SimpleAutoDrive extends Command {

    private final DriveSubsystem drive;
    private final boolean xDirection;
    private final int direction;

    private final double totalDecelerationTime;
    private double fullSpeedTime;
    private double currentDecelerationTime;

    private double previousTime;

    /**
     * Construct a new command to drive a given distance in a given direction. The
     * command will ramp down at the end to increase accuracy.
     * 
     * @param drive     The drive subsystem
     * @param distance  how far the robot should drive in meters
     * @param x         True if this command drives in the x direction, false if it
     *                  drives in the y direction
     * @param direction The direction of travel, based on the sign
     */
    public SimpleAutoDrive(DriveSubsystem drive, double distance, boolean x, int direction) {
        this.drive = drive;
        this.xDirection = x;
        this.direction = (int) Math.copySign(1, direction);

        //Spend 10 percent of the distance slowing down
        fullSpeedTime = ( (distance * 0.9) / Constants.Drive.AUTONOMOUS_VELOCITY);
        //Spend however long it takes to slow down from top speed to zero
        totalDecelerationTime = Constants.Drive.AUTONOMOUS_VELOCITY / Constants.Drive.AUTONOMOUS_ACCELERATION;
        currentDecelerationTime = totalDecelerationTime;
    }

    @Override
    public void initialize() {
        previousTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (fullSpeedTime > 0) {
            drive.singleDirectionDrive(Constants.Drive.AUTONOMOUS_VELOCITY * direction, xDirection);
            fullSpeedTime = fullSpeedTime - getTimeDelta();
        } else {
            //Interpolate between max velocity and zero to get current velocity
            drive.singleDirectionDrive(
                (Constants.Drive.AUTONOMOUS_VELOCITY * (currentDecelerationTime / totalDecelerationTime)) * direction, 
                xDirection
            );

            currentDecelerationTime = currentDecelerationTime - getTimeDelta();
        }

        previousTime = Timer.getFPGATimestamp();
    }

    private double getTimeDelta() {
        return Timer.getFPGATimestamp() - previousTime;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return currentDecelerationTime <= 0;
    }

}
