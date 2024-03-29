package frc.robot.auto.generator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto.cmd.arm.AutoArmHome;
import frc.robot.auto.cmd.arm.AutoArmMove;
import frc.robot.auto.cmd.group.AutoIntakeNote;
import frc.robot.auto.cmd.group.ConfigurableShootSequence;
import frc.robot.auto.cmd.group.ExtraPathfinding;
import frc.robot.auto.cmd.group.ShootSequence;
import frc.robot.auto.cmd.shooter.AutoSetTargetSpeed;
import frc.robot.auto.cmd.shooter.AutoShoot;
import frc.robot.auto.cmd.shooter.AutoSpinUp;
import frc.robot.drive.DriveSubsystem;
import frc.robot.notification.LEDSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.SpinUpAndShoot;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AllianceUtil;

public class PathPlannerAutoGenerator {

    private SendableChooser<Command> autoSelector;
    private SendableChooser<Pose2d> extraPathfindPoses;

    private Field2d field;
    private ShuffleboardTab tab;
    private GenericEntry delay;

    private final DriveSubsystem drive;
    private final ArmSubsystem arm;
    private final ShooterSubsystem shooter;
    private final UndertakerSubsystem undertaker;
    private final LEDSubsystem led;
    private Command ppAuto;

    private final AutonomousContext autoContext;

    public PathPlannerAutoGenerator(DriveSubsystem drive, ArmSubsystem arm, ShooterSubsystem shooter, UndertakerSubsystem undertaker, LEDSubsystem led) {
        this.drive = drive;
        this.arm = arm;
        this.shooter = shooter;
        this.undertaker = undertaker;
        this.led = led;

        this.autoContext = new AutonomousContext(Constants.Arm.HOME_POSITION);

        registerCommands();

        AutoBuilder.configureHolonomic(
            drive::getPose,
            drive::resetOdometry,
            drive::getRobotRelativeSpeeds,
            drive::setRobotRelativeChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.Auto.TRANSLATION_P, Constants.Auto.TRANSLATION_I, Constants.Auto.TRANSLATION_D),
                new PIDConstants(Constants.Auto.ROTATION_P, Constants.Auto.ROTATION_I, Constants.Auto.ROTATION_D),
                Constants.Auto.MODULE_MAXIMUM_VELOCITY, 
                Constants.Auto.DRIVEBASE_RADIUS, 
                new ReplanningConfig(true, false)
            ),
            AllianceUtil::isRedAlliance,
            drive
        );


        autoSelector = AutoBuilder.buildAutoChooser(); // the sendablechooser should really only be the name strings (fix w/ jake later)
        autoSelector.onChange((cmd) -> {
            if(cmd != null){
                setStartingPose(cmd.getName());
                ppAuto = new PathPlannerAuto(cmd.getName()); // because we are composing later, we need to cook a fresh Command every time we select.
            }
        });

        extraPathfindPoses = new SendableChooser<>();
        setupExtraPathfindingPoses();

        setupShuffleboard();
    }

    private void setStartingPose(String cmdName) {
        if (cmdName == null || cmdName.equals("InstantCommand")) {
            //This is the none command
            field.setRobotPose(new Pose2d());
            return;
        }

        try {
            field.setRobotPose(PathPlannerAuto.getStaringPoseFromAutoFile(cmdName));
        } catch(Exception e) {
            DataLogManager.log("An exception occurred while setting the starting position of a path planner path: " + e.getMessage());
            field.setRobotPose(new Pose2d());
        }
        
    }

    private void setupExtraPathfindingPoses() {
        extraPathfindPoses.setDefaultOption("None", new Pose2d(-1, -1, Rotation2d.fromDegrees(0)));
        if (AllianceUtil.isRedAlliance()) {
            extraPathfindPoses.addOption("Amp", new Pose2d(14.5, 7.6, Rotation2d.fromDegrees(0)));
        }else {
            extraPathfindPoses.addOption("Amp", new Pose2d(1.9, 7.6, Rotation2d.fromDegrees(0)));
        }
        extraPathfindPoses.addOption("Center Note", new Pose2d(8.3, 4.1, Rotation2d.fromDegrees(0)));
        //TODO more poses and correct rotation
    }

    private void setupShuffleboard() {
        field = new Field2d();
        tab = Shuffleboard.getTab("Auto");
        tab.add("Auto Selection", autoSelector).withPosition(0, 0).withSize(2, 1);
        tab.add("Starting Pose", field).withPosition(0, 1).withSize(6, 4);
        tab.addString("Alliance", () -> AllianceUtil.isRedAlliance() ? "Red" : "Blue").withPosition(6, 1);
        delay = tab.add("Auto delay seconds", 0).withSize(2, 1).withPosition(3, 0).getEntry();
        tab.add("Extra pathfind", extraPathfindPoses).withSize(2, 1).withPosition(5, 0);
    }

    private void registerCommands() {
        NamedCommands.registerCommand("shootSequence", new ShootSequence(shooter, undertaker, arm, led, autoContext));
        
        NamedCommands.registerCommand("closeAShot", new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.CLOSE_A_SHOT_ANGLE));
        NamedCommands.registerCommand("closeBShot", new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.CLOSE_B_SHOT_ANGLE));
        NamedCommands.registerCommand("closeCShot", new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.CLOSE_C_SHOT_ANGLE));
        NamedCommands.registerCommand("closerCShot", new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.CLOSER_C_SHOT_ANGLE));
        NamedCommands.registerCommand("behindCloseBShot", new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.BEHIND_CLOSE_B_SHOT_ANGLE));
        NamedCommands.registerCommand("sourceSideShot", new ConfigurableShootSequence(shooter, undertaker, arm, led, () -> Constants.Auto.SOURCE_SIDE_SHOT_ANGLE));

        NamedCommands.registerCommand("intake", new AutoIntakeNote(shooter, undertaker)); //blocking
        NamedCommands.registerCommand("intakeUntil", new AutoIntakeNote(shooter, undertaker).withTimeout(0.5)); //blocking

        NamedCommands.registerCommand("armHome", new AutoArmHome(arm)); //blocking

        NamedCommands.registerCommand("armNoteLeft", new AutoArmMove(arm, Constants.Auto.CLOSE_A_SHOT_ANGLE)); //blocking
        NamedCommands.registerCommand("armNoteMiddle", new AutoArmMove(arm, Constants.Auto.CLOSE_B_SHOT_ANGLE)); //blocking
        NamedCommands.registerCommand("armNoteRight", new AutoArmMove(arm, Constants.Auto.CLOSER_C_SHOT_ANGLE)); //blocking
        
        NamedCommands.registerCommand("shoot", new AutoShoot(shooter).withTimeout(0.25)); //blocking
        NamedCommands.registerCommand("spinUpClose", new AutoSpinUp(shooter, Constants.Auto.CLOSE_SHOT_SPEED_TOP, Constants.Auto.CLOSE_SHOT_SPEED_BOTTOM)); //blocking
        NamedCommands.registerCommand("spinUpFar", new AutoSetTargetSpeed(shooter, Constants.Auto.SHOT_SPEED, Constants.Auto.SHOT_SPEED)); //non-blocking
        NamedCommands.registerCommand("spinDown", new AutoSetTargetSpeed(shooter, 0, 0)); //non-blocking
    }

    private void setupAutoContext(String selectedAuto) {
        autoContext.clear();
        DataLogManager.log("setting auto context for " + selectedAuto);
        switch (selectedAuto) {
            case "SubAClose4":
                autoContext.addAngle(Constants.Auto.CLOSE_A_SHOT_ANGLE, Constants.Auto.CLOSE_B_SHOT_ANGLE, Constants.Auto.CLOSER_C_SHOT_ANGLE);
                break;
            case "SubB2":
                autoContext.addAngle(Constants.Auto.CLOSE_B_SHOT_ANGLE);
                break;
            case "AmpSide3":
                autoContext.addAngle(Constants.Auto.CLOSE_A_SHOT_ANGLE, Constants.Auto.CLOSE_A_SHOT_ANGLE);
                break;
            case "SubCMidC2":
                autoContext.addAngle(Constants.Auto.FAR_SHOT_ANGLE);
                break;
        }
    }

    public Command getAutoCommand() {
        setupAutoContext(autoSelector.getSelected().getName());
        Command autoCmd = new SequentialCommandGroup(
            //We always want to shoot the preloaded piece, and we want to shoot before the delay.
            new SpinUpAndShoot(shooter, () -> Constants.Shooter.SHOOT_SPEED, () -> Constants.Shooter.SHOOT_SPEED * 0.9),

            //If a delay is set in the shuffleboard, wait that long
            //This has strategic value and is not required for technical reasons. 
            new WaitCommand(delay.getDouble(0)), 
            ppAuto,
            new AutoSetTargetSpeed(shooter, 0, 0),
            new ExtraPathfinding(shooter, undertaker, led, extraPathfindPoses::getSelected, drive::getPose)
        );
        ppAuto = new PathPlannerAuto(autoSelector.getSelected().getName()); // rebake pp auto... this might cause delay (bad)
        return autoCmd;
    }

}
