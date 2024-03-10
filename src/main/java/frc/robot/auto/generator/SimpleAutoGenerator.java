package frc.robot.auto.generator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.auto.simple.SubwooferFrontScore;
import frc.robot.auto.simple.SubwooferLeftScore;
import frc.robot.auto.simple.SubwooferRightScore;
import frc.robot.drive.DriveSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.RobotContext;

public class SimpleAutoGenerator {

    private final SendableChooser<String> autoSelector;
    private final Field2d field;

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final UndertakerSubsystem undertaker;
    private final RobotContext context;

    public SimpleAutoGenerator(DriveSubsystem drive, ShooterSubsystem shooter, UndertakerSubsystem undertaker, RobotContext context) {
        this.drive = drive;
        this.shooter = shooter;
        this.undertaker = undertaker;
        this.context = context;

        autoSelector = new SendableChooser<String>();
        field = new Field2d();

        initShuffleboardTab();
    }

    private void initShuffleboardTab() {
        autoSelector.setDefaultOption("None", "none");
        autoSelector.addOption("Start from subwoofer front, shoot preloaded, exit with undertaker running", "front");
        autoSelector.addOption("Start from subwoofer left, shoot preloaded, exit with undertaker running", "left");
        autoSelector.addOption("Start from subwoofer right, shoot preloaded, exit with undertaker running", "right");

        ShuffleboardTab tab = Shuffleboard.getTab("Auto");

        tab.add("Auto Selection", autoSelector).withPosition(0, 0).withSize(5, 1);
        tab.add("Starting Pose", field).withPosition(0, 1).withSize(6, 4);

        autoSelector.onChange((String val) -> updateMap());
    }

    public Command getAutoCommand() {
        switch (autoSelector.getSelected()) {
            case "front":
                return new SubwooferFrontScore(drive, shooter, undertaker, context);
            case "left":
                return new SubwooferLeftScore(drive, shooter, undertaker, context);
            case "right":
                return new SubwooferRightScore(drive, shooter, undertaker, context);
            default:
                DataLogManager.log("No autonomous routine selected. Nothing will happen during the autonomous period.");
                return new PrintCommand("No autonomous selected");
        }
    }

    private void updateMap() {
        Pose2d pose;
        switch (autoSelector.getSelected()) {
            case "front":
                pose = Constants.Field.SUBWOOFER_FRONT;
                break;
            case "left":
                pose = Constants.Field.SUBWOOFER_LEFT;
                break;
            case "right":
                pose = Constants.Field.SUBWOOFER_RIGHT;
                break;
            default:
                pose = new Pose2d();
        }

        field.getRobotObject().setPose(
            pose.getX() + 1, //the field visual on shuffleboard is offset slightly (not sure why)
            pose.getY(), 
            pose.getRotation()
        );
    }

}
