package frc.robot.auto.generator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto.cmd.arm.AutoArmHome;
import frc.robot.auto.cmd.group.AutoIntakeNote;
import frc.robot.auto.cmd.group.ConfigurableShootSequence;
import frc.robot.auto.cmd.shooter.AutoSetTargetSpeed;
import frc.robot.auto.cmd.shooter.AutoSpinUp;
import frc.robot.drive.DriveSubsystem;
import frc.robot.notification.LEDSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.SpinUpAndShoot;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AllianceUtil;
import java.util.Collections;
import java.util.List;

public class PathPlannerAutoGenerator {

    private SendableChooser<String> autoSelector;

    private Field2d field;
    private ShuffleboardTab tab;
    private GenericEntry delay;

    private final ArmSubsystem arm;
    private final ShooterSubsystem shooter;
    private final UndertakerSubsystem undertaker;
    private final LEDSubsystem led;
    private Command ppAuto;

    public PathPlannerAutoGenerator(
            DriveSubsystem drive,
            ArmSubsystem arm,
            ShooterSubsystem shooter,
            UndertakerSubsystem undertaker,
            LEDSubsystem led) {
        this.arm = arm;
        this.shooter = shooter;
        this.undertaker = undertaker;
        this.led = led;

        registerCommands();

        AutoBuilder.configureHolonomic(
                drive::getPose,
                drive::resetOdometry,
                drive::getRobotRelativeSpeeds,
                drive::setRobotRelativeChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(
                                Constants.Auto.TRANSLATION_P,
                                Constants.Auto.TRANSLATION_I,
                                Constants.Auto.TRANSLATION_D),
                        new PIDConstants(
                                Constants.Auto.ROTATION_P, Constants.Auto.ROTATION_I, Constants.Auto.ROTATION_D),
                        Constants.Auto.MODULE_MAXIMUM_VELOCITY,
                        Constants.Auto.DRIVEBASE_RADIUS,
                        new ReplanningConfig(true, false)),
                AllianceUtil::isRedAlliance,
                drive);

        autoSelector = new SendableChooser<String>();
        List<String> autos = AutoBuilder.getAllAutoNames();
        Collections.sort(autos);

        for (String s : autos) {
            autoSelector.addOption(s, s);
        }
        autoSelector.setDefaultOption("None", "None");

        autoSelector.onChange((cmd) -> {
            if (cmd != null) {
                setStartingPose(cmd);
                ppAuto = getPathPlannerAuto(); // because we are composing later, we need to cook a fresh
                // Command every time we select.
            }
        });

        setupShuffleboard();
    }

    private void setStartingPose(String cmdName) {
        if (cmdName == null || cmdName.equals("None")) {
            field.setRobotPose(new Pose2d());
            return;
        }

        try {
            field.setRobotPose(PathPlannerAuto.getStaringPoseFromAutoFile(cmdName));
        } catch (Exception e) {
            DataLogManager.log("An exception occurred while setting the starting position of a path planner path: "
                    + e.getMessage());
            field.setRobotPose(new Pose2d());
        }
    }

    private void setupShuffleboard() {
        field = new Field2d();
        tab = Shuffleboard.getTab("Auto");
        tab.add("Auto Selection", autoSelector).withPosition(0, 0).withSize(2, 1);
        tab.add("Starting Pose", field).withPosition(0, 1).withSize(6, 4);
        tab.addString("Alliance", () -> AllianceUtil.isRedAlliance() ? "Red" : "Blue")
                .withPosition(6, 1);
        delay = tab.add("Auto delay seconds", 0)
                .withSize(2, 1)
                .withPosition(3, 0)
                .getEntry();
    }

    private void registerCommands() {
        NamedCommands.registerCommand(
                "closeAShot",
                new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.CLOSE_A_SHOT_ANGLE));
        NamedCommands.registerCommand(
                "closeBShot",
                new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.CLOSE_B_SHOT_ANGLE));
        NamedCommands.registerCommand(
                "closeCShot",
                new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.CLOSE_C_SHOT_ANGLE));
        NamedCommands.registerCommand(
                "closerCShot",
                new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.CLOSER_C_SHOT_ANGLE));
        NamedCommands.registerCommand(
                "behindCloseBShot",
                new ConfigurableShootSequence(
                        shooter, undertaker, arm, led, () -> Constants.Auto.BEHIND_CLOSE_B_SHOT_ANGLE));
        NamedCommands.registerCommand(
                "sourceSideShot",
                new ConfigurableShootSequence(
                        shooter, undertaker, arm, led, () -> Constants.Auto.SOURCE_SIDE_SHOT_ANGLE));

        NamedCommands.registerCommand("intake", new AutoIntakeNote(shooter, undertaker)); // blocking
        NamedCommands.registerCommand(
                "intakeUntil", new AutoIntakeNote(shooter, undertaker).withTimeout(0.5)); // blocking

        NamedCommands.registerCommand("armHome", new AutoArmHome(arm)); // blocking

        NamedCommands.registerCommand(
                "spinUpClose",
                new AutoSpinUp(
                        shooter,
                        Constants.Auto.CLOSE_SHOT_SPEED_TOP,
                        Constants.Auto.CLOSE_SHOT_SPEED_BOTTOM)); // blocking
        NamedCommands.registerCommand(
                "spinUpFar",
                new AutoSetTargetSpeed(shooter, Constants.Auto.SHOT_SPEED, Constants.Auto.SHOT_SPEED)); // non-blocking
        NamedCommands.registerCommand("spinDown", new AutoSetTargetSpeed(shooter, 0, 0)); // non-blocking
    }

    private Command getPathPlannerAuto() {
        String selected = autoSelector.getSelected();
        if (selected == null || selected.equalsIgnoreCase("none")) {
            return Commands.none();
        }
        return new PathPlannerAuto(selected);
    }

    public Command getAutoCommand() {
        Command autoCmd = new SequentialCommandGroup(
                // We always want to shoot the preloaded piece, and we want to shoot before the delay.
                new SpinUpAndShoot(
                        shooter, () -> Constants.Shooter.SHOOT_SPEED, () -> Constants.Shooter.SHOOT_SPEED * 0.9),

                // If a delay is set in the shuffleboard, wait that long
                // This has strategic value and is not required for technical reasons.
                new WaitCommand(delay.getDouble(0)),
                ppAuto,
                new AutoSetTargetSpeed(shooter, 0, 0));
        ppAuto = getPathPlannerAuto(); // rebake pp auto... this might cause delay (bad)
        return autoCmd;
    }
}
