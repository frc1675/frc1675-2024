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
import frc.robot.auto.cmd.ShootSequence;
import frc.robot.auto.cmd.arm.AutoArmHome;
import frc.robot.auto.cmd.arm.AutoArmMove;
import frc.robot.auto.cmd.shooter.AutoSetTargetSpeed;
import frc.robot.auto.cmd.shooter.AutoShoot;
import frc.robot.auto.cmd.shooter.AutoSpinUp;
import frc.robot.drive.DriveSubsystem;
import frc.robot.notification.LEDSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.SpinUpAndShoot;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AllianceUtil;

public class PathPlannerAutoGenerator extends AbstractAutoGenerator {

    private SendableChooser<Command> autoSelector;

    private final ArmSubsystem arm;
    private final ShooterSubsystem shooter;
    private final UndertakerSubsystem undertaker;
    private final LEDSubsystem led;

    private final AutonomousContext autoContext;

    public PathPlannerAutoGenerator(DriveSubsystem drive, ArmSubsystem arm, ShooterSubsystem shooter, UndertakerSubsystem undertaker, LEDSubsystem led) {
        super("PathPlanner");
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
        NamedCommands.registerCommand("shootSequence", new ShootSequence(shooter, undertaker, arm, led, autoContext));

        NamedCommands.registerCommand("intake", new AutoIntakeNote(shooter, undertaker)); //blocking
        NamedCommands.registerCommand("intakeUntil", new AutoIntakeNote(shooter, undertaker).withTimeout(0.5)); //blocking

        NamedCommands.registerCommand("armHome", new AutoArmHome(arm)); //blocking

        NamedCommands.registerCommand("armNoteLeft", new AutoArmMove(arm, Constants.Auto.LEFT_NOTE_ANGLE)); //blocking
        NamedCommands.registerCommand("armNoteMiddle", new AutoArmMove(arm, Constants.Auto.MIDDLE_NOTE_ANGLE)); //blocking
        NamedCommands.registerCommand("armNoteRight", new AutoArmMove(arm, Constants.Auto.RIGHT_NOTE_ANGLE)); //blocking
        
        NamedCommands.registerCommand("shoot", new AutoShoot(shooter).withTimeout(0.25)); //blocking
        NamedCommands.registerCommand("spinUpClose", new AutoSpinUp(shooter, Constants.Auto.CLOSE_SHOT_SPEED_TOP, Constants.Auto.CLOSE_SHOT_SPEED_BOTTOM)); //blocking
        NamedCommands.registerCommand("spinUpFar", new AutoSetTargetSpeed(shooter, Constants.Auto.SHOT_SPEED, Constants.Auto.SHOT_SPEED)); //non-blocking
        NamedCommands.registerCommand("spinDown", new AutoSetTargetSpeed(shooter, 0, 0)); //non-blocking
    }

    private void setupAutoContext(String selectedAuto) {
        autoContext.clear();
        switch (selectedAuto) {
            case "Score4":
                autoContext.addAngle(Constants.Auto.LEFT_NOTE_ANGLE, Constants.Auto.MIDDLE_NOTE_ANGLE, Constants.Auto.RIGHT_NOTE_ANGLE);
                break;
            case "CenterClose":
                autoContext.addAngle(Constants.Auto.MIDDLE_NOTE_ANGLE);
                break;
            case "LeftFarLane":
                autoContext.addAngle(Constants.Auto.LEFT_NOTE_ANGLE, Constants.Auto.LEFT_NOTE_ANGLE);
                break;
            case "RightFarLane":
                autoContext.addAngle(Constants.Auto.FAR_SHOT_ANGLE);
                break;
        }
    }

    @Override
    public Command getAutoCommand() {
        setupAutoContext(autoSelector.getSelected().getName());
        return new SequentialCommandGroup(
            //We always want to shoot the preloaded piece, and we want to shoot before the delay.
            new SpinUpAndShoot(shooter, () -> Constants.Shooter.SHOOT_SPEED, () -> Constants.Shooter.SHOOT_SPEED * 0.9),

            //If a delay is set in the shuffleboard, wait that long
            //This has strategic value and is not required for technical reasons. 
            new WaitCommand(this.getDelay(0)), 
            autoSelector.getSelected(),
            new AutoSetTargetSpeed(shooter, 0, 0)
        );
    }

}
