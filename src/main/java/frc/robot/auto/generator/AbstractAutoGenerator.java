package frc.robot.auto.generator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class AbstractAutoGenerator {
    private final Field2d field;
    private final ShuffleboardTab tab;

    public AbstractAutoGenerator(String mode) {
        field = new Field2d();
        tab = Shuffleboard.getTab("Auto");
        tab.add("Starting Pose", field).withPosition(0, 1).withSize(6, 4);
        tab.addString("Mode", () -> mode).withPosition(5, 0);
    }

    protected final void setFieldPose(Pose2d pose) {
        field.getRobotObject().setPose(
            pose.getX() + 1, //the field visual on shuffleboard is offset slightly (not sure why)
            pose.getY(), 
            pose.getRotation()
        );
    }

    protected final ShuffleboardTab getTab() {
        return tab;
    }

    public abstract Command getAutoCommand();

}
