package frc.robot.auto.generator;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.drive.DriveSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.Shoot;
import frc.robot.shooter.commands.SpinDown;
import frc.robot.shooter.commands.SpinUp;
import frc.robot.undertaker.UndertakerIntake;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.RobotContext;

public class AutoGenerator {

    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
        Constants.PathPlanner.MAXIMUM_VELOCITY, 
        Constants.PathPlanner.MAXIMUM_ACCELERATION,
        Constants.PathPlanner.MAXIMUM_ANGULAR_VELOCITY,
        Constants.PathPlanner.MAXIMUM_ANGULAR_ACCELERATION
    );

    private SendableChooser<Command> autoSelector;
    private Field2d fieldMap = new Field2d();

    private final DriveSubsystem drive;
    private final Arm arm;
    private final ShooterSubsystem shooter;
    private final UndertakerSubsystem undertaker;
    private final RobotContext robotContext;

    private String previousPath = "";

    public AutoGenerator(DriveSubsystem drive, Arm arm, ShooterSubsystem shooter, UndertakerSubsystem undertaker, RobotContext robotContext) {
        this.drive = drive;
        this.arm = arm;
        this.shooter = shooter;
        this.undertaker = undertaker;
        this.robotContext = robotContext;

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
            () -> Robot.isSimulation() ? false : DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red),
            drive
        );

        registerCommands();
        initShuffleboardTab();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("armHome", new MoveToHome(arm));
        NamedCommands.registerCommand("armAmp", new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
        NamedCommands.registerCommand("spinUp", new SpinUp(shooter, robotContext::shouldSlowShoot));
        NamedCommands.registerCommand("shoot", new Shoot(shooter).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME));
        NamedCommands.registerCommand("spinDown", new SpinDown(shooter));
        NamedCommands.registerCommand("runUndertaker", new UndertakerIntake(undertaker, robotContext::getReadyToIntake));
        NamedCommands.registerCommand("disableUndertaker", new InstantCommand(() -> robotContext.setIntakeEnabledOverride(false)));
        NamedCommands.registerCommand("enableUndertaker", new InstantCommand(() -> robotContext.setIntakeEnabledOverride(true)));
    }

    private void initShuffleboardTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");

        autoSelector = AutoBuilder.buildAutoChooser();

        tab.add("Auto Selection", autoSelector).withPosition(0, 0).withSize(2, 1);
        tab.add("Selected Path", fieldMap).withPosition(0, 1).withSize(5, 3);
    }

    public Command getAutoCommand() {
        return autoSelector.getSelected();
    }

    public boolean canFollowPath(Pose2d endPose, Pose2d...additionalPoses) {
        Pose2d currentPose = drive.getPose();
        if (endPose.getTranslation().getDistance(currentPose.getTranslation()) > Constants.PathPlanner.DYNAMIC_PATHING_MAX_DISTANCE) {
            return false;
        }

        for (Pose2d pose : additionalPoses) {
            if (pose.getTranslation().getDistance(currentPose.getTranslation()) > Constants.PathPlanner.DYNAMIC_PATHING_MAX_DISTANCE) {
                return false;
            }
        }

        return true;
    }

    /**
     * Dynamically generates a command. The command runs a path which moves through
     * the given poses,
     * starting at the current pose. Note that the rotation is the direction of
     * travel, not the holonomic (swerve) rotation.
     * 
     * @param endRotation     the holonomic (swerve) rotation at the end of the path
     * @param endPose         the target end pose
     * @param additionalPoses other points the robot should travel through before
     *                        reaching the end pose. This is optional.
     * @return a command which moves through all of the given points, ending
     *         at the endPose, rotated to the target endRotation, with a velocity of
     *         zero.
     */
    public Command generatePathFollowCommand(Rotation2d endRotation, Pose2d endPose, Pose2d...additionalPoses) {
        Pose2d currentPose = drive.getPose();

        if (!canFollowPath(endPose, additionalPoses)) {
            DataLogManager.log("Refused to generate path as robot too far away from goal end pose.");
            return Commands.none();
        }

        List<Pose2d> poses = new ArrayList<Pose2d>();
        poses.add(currentPose); //Path should start where the robot is right now
        for (Pose2d pose : additionalPoses) {
            poses.add(pose);
        }
        poses.add(endPose);

        List<Translation2d> points = PathPlannerPath.bezierFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(points, DEFAULT_PATH_CONSTRAINTS, new GoalEndState(0, endRotation));

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }


    public void updateMap() {
        List<State> allStates = new ArrayList<State>();
        boolean selectedCommandExists = true;

        try {
            PathPlannerAuto.getPathGroupFromAutoFile(getAutoCommand().getName());
        } catch (RuntimeException e) {
            selectedCommandExists = false;
        }
        
        if (!getAutoCommand().getName().equals(previousPath)) {
            if(selectedCommandExists) {
            for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(getAutoCommand().getName())) {
                allStates.addAll(path.getTrajectory(new ChassisSpeeds(), new Rotation2d()).getStates());
            }

            Trajectory trajectory = statesToWPITrajectory(allStates);

            fieldMap.setRobotPose(trajectory.getInitialPose());
            fieldMap.getObject("traj").setTrajectory(trajectory);
            }else {
                fieldMap.setRobotPose(0, 0, Rotation2d.fromDegrees(0));
                fieldMap.getObject("traj").setTrajectory(new Trajectory());
            }
        }
        previousPath = getAutoCommand().getName();
    }

    private Trajectory statesToWPITrajectory(List<State> states) { 
        List<edu.wpi.first.math.trajectory.Trajectory.State> rtn = new ArrayList<edu.wpi.first.math.trajectory.Trajectory.State>(states.size());

        for (int i = 0; i < states.size(); i++) {
            rtn.add(
                new edu.wpi.first.math.trajectory.Trajectory.State(
                    states.get(i).timeSeconds, 
                    states.get(i).velocityMps, 
                    states.get(i).accelerationMpsSq, 
                    new Pose2d(states.get(i).positionMeters, states.get(i).targetHolonomicRotation),
                    states.get(i).curvatureRadPerMeter
                )
            );
        }
 
        return new Trajectory(rtn);
    }

}
