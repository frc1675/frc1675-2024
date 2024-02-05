package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.drive.DriveSubsystem;

public class AutoGenerator {

    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
        Constants.PathPlanner.MAXIMUM_VELOCITY, 
        Constants.PathPlanner.MAXIMUM_ACCELERATION,
        Constants.PathPlanner.MAXIMUM_ANGULAR_VELOCITY,
        Constants.PathPlanner.MAXIMUM_ANGULAR_ACCELERATION
    );

    private SendableChooser<Command> autoSelector;
    private Field2d fieldMap = new Field2d();

    public AutoGenerator(DriveSubsystem drive) {
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
                new ReplanningConfig(false, false)
            ),
            () -> allianceIsRed(),
            drive
        );

        registerCommands();
        initShuffleboardTab();
    }

    private boolean allianceIsRed() {
        if(Robot.isSimulation()) {
            return false;
        }
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    private void registerCommands() {
        NamedCommands.registerCommand("notePickup", new PrintCommand("Note pickup command triggered!"));
        NamedCommands.registerCommand("scoreHigh", new PrintCommand("Score high command triggered!"));
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


    public void updateMap() {
        List<State> allStates = new ArrayList<State>();
        boolean selectedCommandExists = true;

        try {
            PathPlannerAuto.getPathGroupFromAutoFile(getAutoCommand().getName());
        } catch (RuntimeException e) {
            selectedCommandExists = false;
        }
        
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

    private Trajectory statesToWPITrajectory(List<State> states) { 
        List<edu.wpi.first.math.trajectory.Trajectory.State> rtn = new ArrayList<edu.wpi.first.math.trajectory.Trajectory.State>(states.size());

        for (int i = 0; i < states.size(); i++) {
            rtn.add(
                new edu.wpi.first.math.trajectory.Trajectory.State(
                    states.get(i).timeSeconds, 
                    states.get(i).velocityMps, 
                    states.get(i).accelerationMpsSq, 
                    new Pose2d(states.get(i).positionMeters.plus(new Translation2d(1, 0)), states.get(i).targetHolonomicRotation),
                    states.get(i).curvatureRadPerMeter
                )
            );
        }
 
        return new Trajectory(rtn);
    }

}
