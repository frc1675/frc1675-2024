package frc.robot.auto.generator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.cmdGroup.IntakeNote;
import frc.robot.cmdGroup.PodiumShot;
import frc.robot.drive.DriveSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.SpinDown;
import frc.robot.shooter.commands.SpinUpAndShoot;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AllianceUtil;
import frc.robot.util.RobotContext;

public class PathPlannerAutoGenerator extends AbstractAutoGenerator {

    private SendableChooser<Command> autoSelector;

    private final ArmSubsystem arm;
    private final ShooterSubsystem shooter;
    private final UndertakerSubsystem undertaker;
    private final RobotContext robotContext;

    public PathPlannerAutoGenerator(DriveSubsystem drive, ArmSubsystem arm, ShooterSubsystem shooter, UndertakerSubsystem undertaker, RobotContext robotContext) {
        super("PathPlanner");
        this.arm = arm;
        this.shooter = shooter;
        this.undertaker = undertaker;
        this.robotContext = robotContext;

        registerCommands();

        AutoBuilder.configureHolonomic(
            drive::getPose,
            drive::resetOdometry,
            drive::getRobotRelativeSpeeds,
            drive::setRobotRelativeChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.PathPlanner.TRANSLATION_P),
                new PIDConstants(Constants.PathPlanner.ROTATION_P),
                Constants.PathPlanner.MAXIMUM_VELOCITY, 
                Constants.PathPlanner.DRIVEBASE_RADIUS, 
                new ReplanningConfig(true, false)
            ),
            AllianceUtil::isRedAlliance,
            drive
        );

        autoSelector = AutoBuilder.buildAutoChooser();
        getTab().add("Auto Selection", autoSelector).withPosition(0, 0).withSize(2, 1);

        autoSelector.onChange( (cmd) -> setStartingPose(cmd == null ? "null" : cmd.getName()));

        FollowPathCommand.warmupCommand().schedule(); //Load all pathplanner classes in order to prevent delay when initally following path
    }

    private void setStartingPose(String cmdName) {
        if (cmdName.equals("InstantCommand") || cmdName.equals("null")) {
            //This is the none command
            setFieldPose(new Pose2d());
            return;
        }

        try {
            setFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile(cmdName));
        } catch(Exception e) {
            DataLogManager.log("An exception occurred while setting the starting position of a path planner path: " + e.getMessage());
            setFieldPose(new Pose2d());
        }
        
    }

    private void registerCommands() {
        NamedCommands.registerCommand("armHome", new MoveToHome(arm));
        NamedCommands.registerCommand("spinUpAndShoot", new SpinUpAndShoot(shooter, 
            () -> robotContext.getShooterSpeed()[0], 
            () -> robotContext.getShooterSpeed()[1]
        ));
        NamedCommands.registerCommand("podiumShot", new PodiumShot(shooter, arm));
        NamedCommands.registerCommand("spinDown", new SpinDown(shooter));
        NamedCommands.registerCommand("runUndertaker", new IntakeNote(shooter, undertaker, robotContext::getReadyToIntake));
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            //If a delay is set in the shuffleboard, wait that long
            //This has strategic value and is not required for technical reasons. 
            new WaitCommand(this.getDelay(0)), 
            autoSelector.getSelected()
        );
    }

}
