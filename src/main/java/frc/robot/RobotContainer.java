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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto.generator.PathPlannerAutoGenerator;
import frc.robot.cmdGroup.IntakeNote;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
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
    private final Controls controlls;
    // private final VisionSubsystem visionSubsystem;
    private final ArmSubsystem arm;

    private final PathPlannerAutoGenerator autoGenerator;
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
        driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);
        operatorController = new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER);
       
        controlls = Controls.create(drive, shooter, poseScheduler, undertakerSubsystem, arm, robotContext, driverController, operatorController);

        autoGenerator = new PathPlannerAutoGenerator(drive, arm, shooter, undertakerSubsystem, ledSubsystem);

        

        Dashboards.initVoltageDashboard();
        Dashboards.initCurrentDashboard();
        Dashboards.initMemoryDashboard();
        Dashboards.initGitInfoDashboard();

        // Comment the below out when not testing.
        // initTestingOnlyTab();

        controlls.driverSelect();
        controlls.operatorSelect();
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
