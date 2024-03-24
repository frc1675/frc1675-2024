package frc.robot.auto.cmd.group;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.notification.AddLEDColor;
import frc.robot.notification.LEDState;
import frc.robot.notification.LEDSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;

public class ExtraPathfinding extends ConditionalCommand {

    public ExtraPathfinding(ShooterSubsystem shooter, UndertakerSubsystem undertaker, LEDSubsystem led, Supplier<Pose2d> selectedPose, Supplier<Pose2d> currentPose) {
        super(
            new SequentialCommandGroup(
                new AddLEDColor(led, LEDState.AUTONOMOUS_PATHFINDING),
                new InstantCommand(() -> DataLogManager.log("Autonomous completed with sufficient time to pathfind. Pathfinding to ." + selectedPose.get().toString())),
                new AutoIntakeNote(shooter, undertaker),
                AutoBuilder.followPath(
                    new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(currentPose.get(), selectedPose.get()),
                        Constants.Auto.EXTRA_PATHFINDING_CONSTRAINTS,
                        new GoalEndState(0, selectedPose.get().getRotation())
                    )
                )
            ),
            new SequentialCommandGroup(
                new AddLEDColor(led, LEDState.AUTONOMOUS_COMPLETE),
                new InstantCommand(() -> {
                    if (selectedPose.get().getX() == -1) {
                        DataLogManager.log("Autonomous completed with sufficient time to pathfind, but no destination pose was selected. Idling until teleop.");
                    }else {
                        DataLogManager.log("Autonomous completed with insufficient time to pathfind. Idling until teleop.");
                    }
                })
            ),
            () -> Timer.getMatchTime() > Constants.Auto.SUFFICIENT_EXTRA_PATHFINDING_TIME && selectedPose.get().getX() != -1 
        );
    }

}
