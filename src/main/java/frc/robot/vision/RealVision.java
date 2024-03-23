package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.vision.VisionSubsystem.LEDMode;

public class RealVision implements IVision {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ups");
    private final NetworkTableEntry aprilTagID = table.getEntry("tid");
    private final NetworkTableEntry isValidTarget = table.getEntry("tv");
    private final NetworkTableEntry ledMode = table.getEntry("LEDMode");
    private LEDMode currentMode;
    private NetworkTableEntry botpose = null;

    public RealVision() {
        ledMode.setNumber(3);
    }

    @Override
    public Pose2d getBotpose() {
        if (botpose instanceof NetworkTableEntry) {
            double[] botposeArray = botpose.getDoubleArray(new double[6]);
            return new Pose2d(botposeArray[0], botposeArray[1], Rotation2d.fromDegrees(botposeArray[5]));
        }
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                botpose = table.getEntry("botpose_wpired");
            } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
                botpose = table.getEntry("botpose_wpiblue");
            } else {
                botpose = table.getEntry("botpose");
            }
            double[] botposeArray = botpose.getDoubleArray(new double[6]);
            return new Pose2d(botposeArray[0], botposeArray[1], Rotation2d.fromDegrees(botposeArray[5]));
        }
        return new Pose2d(-1000, -1000, Rotation2d.fromDegrees(0)); // Out of Bounds Default Pose for Drive Subsystem to
    }

    @Override
    public boolean hasTarget() {
        return isValidTarget.getDouble(0) == 1;
    }

    @Override
    public int getTargetId() {
        return (int) aprilTagID.getInteger(0);
    }

    @Override
    public boolean hasSpeaker() {
        if (hasTarget() && (getTargetId() == Constants.Vision.RED_SPEAKER_TARGET_ID ||
                getTargetId() == Constants.Vision.BLUE_SPEAKER_TARGET_ID))
            return true;
        return false;
    }

    @Override
    public Rotation2d getTargetHorizontalOffset() {
        return Rotation2d.fromDegrees(table.getEntry("tx").getDouble(-1000));
    }

    @Override
    public Rotation2d getTargetVerticalOffset() {
        return Rotation2d.fromDegrees(table.getEntry("ty").getDouble(-1000));
    }

    @Override
    public void setLEDMode(LEDMode ledState) {
        switch (ledState) {
            case OFF:
                ledMode.setNumber(3);
                currentMode = LEDMode.OFF;
                break;
            case BLINK:
                ledMode.setNumber(2);
                currentMode = LEDMode.BLINK;
                break;
            case ON:
                ledMode.setNumber(1);
                currentMode = LEDMode.ON;
                break;
            default:
                DataLogManager.log("I have no clue how this happened?");
                break;
        }
    }

    @Override
    public LEDMode getLEDMode() {
        return currentMode;
    }

    @Override
    public Double getHorizontalDistance() {
        // make sure limelight horizon is calibrated correctly!
        double[] cameraSpace = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        if (cameraSpace.length > 0) {
            Translation2d tagPose = new Translation2d(cameraSpace[0], cameraSpace[1]);
            return new Translation2d(0, 0).getDistance(tagPose); // returns meters
        }
        return -1000.0;
    }
}
