package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RealVision implements IVision {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ups");
    private final NetworkTableEntry aprilTagID = table.getEntry("tid");
    private final NetworkTableEntry isValidTarget = table.getEntry("tv");
    private NetworkTableEntry botpose = null;

    @Override
    public Pose2d getBotpose() {
        if (botpose instanceof NetworkTableEntry) {
            double[] botposeArray = botpose.getDoubleArray(new double[6]);
            return new Pose2d(botposeArray[0], botposeArray[1], Rotation2d.fromDegrees(botposeArray[5]));
        }
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                botpose = table.getEntry("botpose_wpired");
            } else {
                botpose = table.getEntry("botpose_wpiblue");
            }
            double[] botposeArray = botpose.getDoubleArray(new double[6]);
            return new Pose2d(botposeArray[0], botposeArray[1], Rotation2d.fromDegrees(botposeArray[5]));
        }
        return new Pose2d(
                -1000, -1000, Rotation2d.fromDegrees(0)); // Out of Bounds Default Pose for Drive Subsystem to Ignore
    }

    @Override
    public boolean hasTarget() {
        return isValidTarget.getDouble(0) == 1;
    }

    @Override
    public int getTargetId() {
        return (int) aprilTagID.getInteger(0);
    }
}
