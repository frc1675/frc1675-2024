// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.auto.generator.PathPlannerAutoGenerator;
import frc.robot.cmdGroup.IntakeNote;
import frc.robot.cmdGroup.ShootAndReturnHome;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.TurnToAngle;
import frc.robot.notification.ContextualColor;
import frc.robot.notification.LEDSubsystem;
import frc.robot.operation.JakeDriverConfiguration;
import frc.robot.operation.OperationConfiguration;
import frc.robot.operation.WillOperatorConfiguration;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Dashboards;
import frc.robot.util.RobotContext;
import frc.robot.util.VersionFile;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class RobotContainer {
    private final PoseScheduler poseScheduler;
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final LEDSubsystem ledSubsystem;
    private final UndertakerSubsystem undertakerSubsystem;
    // private final VisionSubsystem visionSubsystem;
    private final ArmSubsystem arm;

    private final PathPlannerAutoGenerator autoGenerator;
    private final RobotContext robotContext;

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    private ArrayList<OperationConfiguration> operationConfigs = new ArrayList<>();

    private boolean shotTesting = false;
    private ShuffleboardTab testOnlyTab;
    private GenericEntry testAngleEntry;

    public RobotContainer() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.log("Data log started.");

        poseScheduler = new PoseScheduler();
        drive = new DriveSubsystem(poseScheduler);

        // visionSubsystem = VisionSubsystem.create();
        ledSubsystem = LEDSubsystem.create();
        undertakerSubsystem = UndertakerSubsystem.create();
        shooter = ShooterSubsystem.create();
        arm = ArmSubsystem.create();

        robotContext = new RobotContext(arm, shooter);

        autoGenerator = new PathPlannerAutoGenerator(drive, arm, shooter, undertakerSubsystem, ledSubsystem);

        driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);
        operatorController = new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER);

        Dashboards.initVoltageDashboard();
        Dashboards.initMemoryDashboard();
        VersionFile.getInstance().putToDashboard();

        // Comment the below out when not testing.
        // initTestingOnlyTab();

        initOperationConfigs();
        registerRobotFunctions();
    }

    private void initOperationConfigs() {
        operationConfigs.add(new JakeDriverConfiguration(driverController));
        operationConfigs.add(new WillOperatorConfiguration(operatorController));

        // examples
        // operationConfigs.add(new InvertedRotationDriverConfiguration(driverController));
        // operationConfigs.add(new SCurvedDriverConfiguration(driverController));
        // operationConfigs.add(new OverridesConfiguration(overrideController));

    }

    private void registerRobotFunctions() {
        for (OperationConfiguration opConfig : operationConfigs) {
            opConfig.registerRobotFunctions(this);
        }
    }

    public void teleopInit() {
        shooter.setTargetShooterSpeeds(0, 0); // Spin down after autonomous
        arm.setTarget(Constants.Arm.HOME_POSITION); // Reset arm position after teleop

        for (OperationConfiguration opConfig : operationConfigs) {
            opConfig.registerTeleopFunctions(this);
        }

        shooter.setDefaultCommand(new IntakeNote(shooter, undertakerSubsystem, robotContext::getReadyToIntake));
        ledSubsystem.setDefaultCommand(new ContextualColor(robotContext, ledSubsystem, driverController.getHID()));
    }

    public void registerZeroGyro(Trigger t) {
        t.onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));
    }

    public void registerShootAndGoHome(Trigger t) {
        t.onTrue(new ShootAndReturnHome(shooter, arm, () -> robotContext.getShooterSpeed()[0], () -> robotContext
                .getShooterSpeed()[1]));
    }

    public void registerTurnToSpeakerWall(Trigger t) {
        t.onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 0 : 180));
    }

    public void registerAimFromPodium(Trigger t) {
        t.onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 150 : -30.5)); // TODO alliance switching
    }

    public void registerTurnToAmp(Trigger t) {
        t.onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 90 : -90));
    }

    public void registerArmToAmp(Trigger t) {
        t.onTrue(new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
    }

    public void registerArmToHome(Trigger t) {
        t.onTrue(new MoveToHome(arm));
    }

    public void registerArmToPodiumShot(Trigger t) {
        t.onTrue(new MoveToPosition(arm, Constants.Arm.PODIUM_SHOT_ANGLE));
    }

    public void registerArmToFarShot(Trigger t) {
        t.onTrue(new MoveToPosition(arm, Constants.Arm.BEHIND_NOTE_B_ANGLE));
    }

    public void registerEnableIntake(Trigger t) {
        t.onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(true)));
    }

    public void registerDisableIntake(Trigger t) {
        t.onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(false)));
    }

    public void registerDefaultDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        drive.setDefaultCommand(new DefaultDrive(drive, x, y, rotation, robotContext::getDriveSpeedScale));
    }

    private double getJoystickInput(CommandGenericHID stick, int axe) {
        return MathUtil.applyDeadband(stick.getRawAxis(axe), Constants.Controller.DEADZONE_CONSTANT);
    }

    public Command getAutonomousCommand() {
        return autoGenerator.getAutoCommand();
    }

    private void initTestingOnlyTab() {
        shotTesting = true; // lock to stop null stuff by accident in config bindings
        testOnlyTab = Shuffleboard.getTab("Test Only");
        testAngleEntry = testOnlyTab
                .add("Shot Test Angle", Constants.Auto.CLOSE_B_SHOT_ANGLE)
                .withSize(2, 1)
                .withPosition(3, 0)
                .getEntry();
    }
}
