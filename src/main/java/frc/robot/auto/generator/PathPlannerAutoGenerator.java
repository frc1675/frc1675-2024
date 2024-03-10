package frc.robot.auto.generator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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

public class PathPlannerAutoGenerator extends AbstractAutoGenerator {

    private SendableChooser<Command> autoSelector;

    private final Arm arm;
    private final ShooterSubsystem shooter;
    private final UndertakerSubsystem undertaker;
    private final RobotContext robotContext;

    public PathPlannerAutoGenerator(DriveSubsystem drive, Arm arm, ShooterSubsystem shooter, UndertakerSubsystem undertaker, RobotContext robotContext) {
        super("PathPlanner");
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

        autoSelector = AutoBuilder.buildAutoChooser();
        getTab().add("Auto Selection", autoSelector).withPosition(0, 0).withSize(2, 1);

        autoSelector.onChange( (cmd) -> setStartingPose(cmd.getName()));

        registerCommands();
    }

    private void setStartingPose(String cmdName) {
        if (cmdName.equals("InstantCommand")) {
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
        NamedCommands.registerCommand("armAmp", new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
        NamedCommands.registerCommand("spinUp", new SpinUp(shooter, robotContext::shouldSlowShoot));
        NamedCommands.registerCommand("shoot", new Shoot(shooter).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME));
        NamedCommands.registerCommand("spinDown", new SpinDown(shooter));
        NamedCommands.registerCommand("runUndertaker", new UndertakerIntake(undertaker, robotContext::getReadyToIntake));
        NamedCommands.registerCommand("disableUndertaker", new InstantCommand(() -> robotContext.setIntakeEnabledOverride(false)));
        NamedCommands.registerCommand("enableUndertaker", new InstantCommand(() -> robotContext.setIntakeEnabledOverride(true)));
    }

    @Override
    public Command getAutoCommand() {
        return autoSelector.getSelected();
    }

}
