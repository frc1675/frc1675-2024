package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.auto.cmd.group.ConfigurableShootSequence;
import frc.robot.auto.cmd.shooter.AutoSpinUp;
import frc.robot.cmdGroup.ShootAndReturnHome;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.TurnToAngle;
import frc.robot.notification.LEDSubsystem;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AllianceUtil;
import frc.robot.util.RobotContext;

public class Controls {
    private boolean shotTesting = false;
    private GenericEntry testAngleEntry;
    private PoseScheduler poseScheduler;
    private ShuffleboardTab dashboard;
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private LEDSubsystem ledSubsystem;
    private UndertakerSubsystem undertakerSubsystem;
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private ArmSubsystem arm;
    private RobotContext robotContext;
    private String driver;
    private String operator;



    public static Controls create(DriveSubsystem drive2, ShooterSubsystem shooter, PoseScheduler poseScheduler2, UndertakerSubsystem undertakerSubsystem2, ArmSubsystem arm2, RobotContext robotContext2, CommandXboxController driverController2, CommandXboxController operatorController2) {
        return new Controls(drive2, shooter, poseScheduler2, undertakerSubsystem2, arm2, robotContext2, driverController2, operatorController2);
    }

    public Controls(DriveSubsystem drive2, ShooterSubsystem shooter2, PoseScheduler poseScheduler2, UndertakerSubsystem undertakerSubsystem2, ArmSubsystem arm2, RobotContext robotContext2, CommandXboxController driverController2, CommandXboxController operatorController2){
        
        poseScheduler = poseScheduler2;
        drive = drive2;
        undertakerSubsystem = undertakerSubsystem2;
        shooter = shooter2;
        arm = arm2;
        robotContext = robotContext2;
        driverController = driverController2;
        operatorController = operatorController2;
        driver = "default";
        operator = "default";
    }

    public void driverSelect() {
        if (driver == "default") {
            configureDefaultDriverBindings();
        }

        if (driver == "jason") {
            configureJasonDriverBindings();
        }

        if (driver == "ij") {
            configureIJDriverBindings();
        }
    }

    public void operatorSelect() {
        if (operator == "default") {
            configureDefaultOperatorBindings();
        }

        if (operator == "jason") {
            configureJasonOperatorBindings();
        }

        if (operator == "ij") {
            configureIJOperatorBindings();
        }
    }


    public void configureDefaultDriverBindings() {
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

    public void configureDefaultOperatorBindings() {
        operatorController.leftTrigger().onTrue(new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
        operatorController.rightTrigger().onTrue(new MoveToHome(arm));

        operatorController.x().onTrue(new MoveToPosition(arm, Constants.Arm.PODIUM_SHOT_ANGLE));
        operatorController.b().onTrue(new MoveToPosition(arm, Constants.Arm.BEHIND_NOTE_B_ANGLE));
        operatorController.a().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(true)));
        operatorController.y().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(false)));
    }

    public void configureJasonDriverBindings() {
        driverController.start().onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));

        driverController
                .leftBumper()
                .onTrue(new ShootAndReturnHome(shooter, arm, () -> robotContext.getShooterSpeed()[0], () -> robotContext
                        .getShooterSpeed()[1]));

        driverController.y().onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 0 : 180));
        driverController
                .x()
                .onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 150 : -30.5)); // TODO alliance switching
        driverController.b().onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 90 : -90));
    }

    public void configureJasonOperatorBindings() {
        operatorController.rightBumper().onTrue(new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
        operatorController.leftBumper().onTrue(new MoveToHome(arm));

        operatorController.y().onTrue(new MoveToPosition(arm, Constants.Arm.PODIUM_SHOT_ANGLE));
        operatorController.a().onTrue(new MoveToPosition(arm, Constants.Arm.BEHIND_NOTE_B_ANGLE));
        operatorController.b().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(true)));
        operatorController.x().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(false)));
    }

    public void configureIJDriverBindings() {
        driverController.start().onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));

        driverController
                .rightTrigger()
                .onTrue(new ShootAndReturnHome(shooter, arm, () -> robotContext.getShooterSpeed()[0], () -> robotContext
                        .getShooterSpeed()[1]));

        driverController.x().onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 0 : 180));
        driverController
                .y()
                .onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 150 : -30.5)); // TODO alliance switching
        driverController.b().onTrue(new TurnToAngle(drive, AllianceUtil.isRedAlliance() ? 90 : -90));
    }

    public void configureIJOperatorBindings() {
        operatorController.leftTrigger().onTrue(new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
        operatorController.leftBumper().onTrue(new MoveToHome(arm));

        operatorController.rightTrigger().onTrue(new MoveToPosition(arm, Constants.Arm.PODIUM_SHOT_ANGLE));
        operatorController.rightBumper().onTrue(new MoveToPosition(arm, Constants.Arm.BEHIND_NOTE_B_ANGLE));
        operatorController.a().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(true)));
        operatorController.b().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(false)));
    }
}
