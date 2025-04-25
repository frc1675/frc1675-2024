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
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.auto.cmd.group.ConfigurableShootSequence;
import frc.robot.auto.cmd.shooter.AutoSpinUp;
import frc.robot.cmdGroup.IntakeNote;
import frc.robot.cmdGroup.ShootAndReturnHome;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.TurnToAngle;
import frc.robot.notification.ContextualColor;
import frc.robot.notification.LEDSubsystem;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Dashboards;
import frc.robot.util.RobotContext;

public class RobotContainer {
    private final PoseScheduler poseScheduler;
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final LEDSubsystem ledSubsystem;
    private final UndertakerSubsystem undertakerSubsystem;
    // private final VisionSubsystem visionSubsystem;
    private final ArmSubsystem arm;

    // private final PathPlannerAutoGenerator autoGenerator;
    private final RobotContext robotContext;

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

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

        // autoGenerator = new PathPlannerAutoGenerator(drive, arm, shooter, undertakerSubsystem, ledSubsystem);

        driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);
        operatorController = new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER);

        Dashboards.initVoltageDashboard();
        Dashboards.initCurrentDashboard();
        Dashboards.initMemoryDashboard();
        Dashboards.initGitInfoDashboard();

        // Comment the below out when not testing.
        // initTestingOnlyTab();

        configureBindings();
    }

    private void configureBindings() {
        driverController.start().onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));

        driverController
                .rightBumper()
                .onTrue(new ShootAndReturnHome(shooter, arm, () -> robotContext.getShooterSpeed()[0], () -> robotContext
                        .getShooterSpeed()[1]));

        driverController.a().onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 0 : 180));
        driverController
                .b()
                .onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 150 : -30.5)); // TODO alliance switching
        driverController.x().onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 90 : -90));

        operatorController.leftTrigger().onTrue(new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
        operatorController.rightTrigger().onTrue(new MoveToHome(arm));

        operatorController.x().onTrue(new MoveToPosition(arm, Constants.Arm.PODIUM_SHOT_ANGLE));
        operatorController.b().onTrue(new MoveToPosition(arm, Constants.Arm.BEHIND_NOTE_B_ANGLE));
        operatorController.a().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(true)));
        operatorController.y().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(false)));

        // operatorController.povUp().onTrue(new SpinUp(shooter, Constants.Shooter.LONG_SHOT_SPEED,
        // Constants.Shooter.LONG_SHOT_SPEED));
        // operatorController.povDown().onTrue(new SpinDown(shooter));

        if (shotTesting) {
            driverController
                    .b()
                    .onTrue(new ConfigurableShootSequence(
                            shooter,
                            undertakerSubsystem,
                            arm,
                            ledSubsystem,
                            () -> testAngleEntry.getDouble(Constants.Auto.CLOSE_B_SHOT_ANGLE)));
            driverController.x().onTrue(new AutoSpinUp(shooter, Constants.Auto.SHOT_SPEED, Constants.Auto.SHOT_SPEED));
            driverController.y().onTrue(new AutoSpinUp(shooter, 0, 0));
        }
    }

    public void teleopInit() {
        shooter.setTargetShooterSpeeds(0, 0); // Spin down after autonomous
        arm.setTarget(Constants.Arm.HOME_POSITION); // Reset arm position after teleop

        drive.setDefaultCommand(new DefaultDrive(
                drive,
                () -> AllianceUtil.getTranslationDirection()
                        * getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
                () -> AllianceUtil.getTranslationDirection()
                        * getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
                () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS),
                robotContext::getDriveSpeedScale));
        shooter.setDefaultCommand(new IntakeNote(shooter, undertakerSubsystem, robotContext::getReadyToIntake));
        ledSubsystem.setDefaultCommand(new ContextualColor(robotContext, ledSubsystem, driverController.getHID()));
    }

    private double getJoystickInput(CommandGenericHID stick, int axe) {
        return MathUtil.applyDeadband(stick.getRawAxis(axe), Constants.Controller.DEADZONE_CONSTANT);
    }

    public Command getAutonomousCommand() {
        // return autoGenerator.getAutoCommand();
        return null;
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
