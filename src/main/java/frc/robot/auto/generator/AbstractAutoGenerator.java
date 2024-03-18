package frc.robot.auto.generator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AllianceUtil;

public abstract class AbstractAutoGenerator {
    private final Field2d field;
    private final ShuffleboardTab tab;
    private final GenericEntry delay;

    public AbstractAutoGenerator(String mode) {
        field = new Field2d();
        tab = Shuffleboard.getTab("Auto");
        tab.add("Starting Pose", field).withPosition(0, 1).withSize(6, 4);
        tab.addString("Mode", () -> mode).withPosition(5, 0);
        tab.addString("Alliance", () -> AllianceUtil.isRedAlliance() ? "Red" : "Blue").withPosition(6, 1);
        delay = tab.add("Auto delay seconds", 0).withSize(2, 1).withPosition(6, 0).getEntry();
    }

    protected final void setFieldPose(Pose2d pose) {
        field.getRobotObject().setPose(
            pose.getX(), 
            pose.getY(), 
            pose.getRotation()
        );
    }

    protected final double getDelay(double defaultDelay) {
        return delay.getDouble(defaultDelay);
    }

    protected final ShuffleboardTab getTab() {
        return tab;
    }

    public abstract Command getAutoCommand();

}
