package frc.robot.auto.generator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.auto.cmd.AutoIntakeNote;
import frc.robot.auto.cmd.arm.AutoArmHome;
import frc.robot.auto.cmd.arm.AutoArmMove;
import frc.robot.auto.cmd.shooter.AutoSetTargetSpeed;
import frc.robot.auto.cmd.shooter.AutoShoot;
import frc.robot.auto.cmd.shooter.AutoSpinUp;
import frc.robot.drive.DriveSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.SpinUpAndShoot;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AllianceUtil;

public class PathPlannerAutoGenerator extends AbstractAutoGenerator {

    private SendableChooser<Command> autoSelector;

    private final ArmSubsystem arm;
    private final ShooterSubsystem shooter;
    private final UndertakerSubsystem undertaker;

    public PathPlannerAutoGenerator(DriveSubsystem drive, ArmSubsystem arm, ShooterSubsystem shooter, UndertakerSubsystem undertaker) {
        super("PathPlanner");
        this.arm = arm;
        this.shooter = shooter;
        this.undertaker = undertaker;

        registerCommands();

        AutoBuilder.configureHolonomic(
            drive::getPose,
            drive::resetOdometry,
            drive::getRobotRelativeSpeeds,
            drive::setRobotRelativeChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.PathPlanner.TRANSLATION_P, Constants.PathPlanner.TRANSLATION_I, Constants.PathPlanner.TRANSLATION_D),
                new PIDConstants(Constants.PathPlanner.ROTATION_P, Constants.PathPlanner.ROTATION_I, Constants.PathPlanner.ROTATION_D),
                Constants.PathPlanner.MAXIMUM_VELOCITY, 
                Constants.PathPlanner.DRIVEBASE_RADIUS, 
                new ReplanningConfig(true, false)
            ),
            AllianceUtil::isRedAlliance,
            drive
        );

        autoSelector = AutoBuilder.buildAutoChooser();
        getTab().add("Auto Selection", autoSelector).withPosition(0, 0).withSize(2, 1);

        autoSelector.onChange((cmd) -> setStartingPose(cmd.getName()));
    }

    private void setStartingPose(String cmdName) {
        if (cmdName == null || cmdName.equals("InstantCommand")) {
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
        NamedCommands.registerCommand("intake", new AutoIntakeNote(shooter, undertaker)); //never ends

        NamedCommands.registerCommand("armHome", new AutoArmHome(arm)); //blocking
        NamedCommands.registerCommand("armPodiumAngle", new AutoArmMove(arm, Constants.Arm.LONG_SHOT_ANGLE)); //blocking
        
        NamedCommands.registerCommand("shoot", new AutoShoot(shooter).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME)); //blocking
        NamedCommands.registerCommand("spinUpClose", new AutoSpinUp(shooter, Constants.Shooter.SHOOT_SPEED, Constants.Shooter.SHOOT_SPEED * 0.9)); //blocking
        NamedCommands.registerCommand("spinUpFar", new AutoSetTargetSpeed(shooter, Constants.Shooter.LONG_SHOT_SPEED, Constants.Shooter.LONG_SHOT_SPEED)); //non-blocking
        NamedCommands.registerCommand("spinDown", new AutoSetTargetSpeed(shooter, 0, 0)); //non-blocking
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            //We always want to shoot the preloaded piece, and we want to shoot before the delay.
            new SpinUpAndShoot(shooter, () -> Constants.Shooter.SHOOT_SPEED, () -> Constants.Shooter.SHOOT_SPEED * 0.9),
            
            //If a delay is set in the shuffleboard, wait that long
            //This has strategic value and is not required for technical reasons. 
            new WaitCommand(this.getDelay(0)), 
            autoSelector.getSelected()
        );
    }

}
