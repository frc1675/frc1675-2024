package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.drive.DriveSubsystem;
import frc.robot.poseScheduler.PoseScheduler;

public class VisionDriveSubsystem extends DriveSubsystem {

    private final IVision impl;
    private final PoseScheduler poseScheduler;

    public static VisionDriveSubsystem create(PoseScheduler poseScheduler) {
        return new VisionDriveSubsystem(Robot.isReal() ? new RealVision() : new SimVision(), poseScheduler);
    }

    public VisionDriveSubsystem(IVision implementation, PoseScheduler poseScheduler) {
        impl = implementation;
        this.poseScheduler = poseScheduler;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addBoolean("Found target: ", () -> impl.hasTarget());
        tab.addInteger("Target Id: ", () -> impl.getTargetId());
        tab.addString("BotPose: ", () -> getVisionBotpose().toString());
    }

    public Pose2d getVisionBotpose() {
        Pose2d rtn = impl.getBotpose();
        if (rtn == null) {
            return new Pose2d(new Translation2d(-1675, -1675), Rotation2d.fromDegrees(0));
        }
        return rtn;
    }

    @Override
    public void periodic() {
        super.periodic();
        poseScheduler.updatePose(this.getPose());
        super.addVisionMeasurement(getVisionBotpose(), Timer.getFPGATimestamp()); // TODO correct timestamp
    }
}
