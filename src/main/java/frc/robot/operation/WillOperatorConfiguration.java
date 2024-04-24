package frc.robot.operation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class WillOperatorConfiguration extends AbstractCommandXboxOperationConfiguration {

    public WillOperatorConfiguration(CommandXboxController controller) {
        super(controller);
    }

    @Override
    public void registerRobotFunctions(RobotContainer rc) {
        rc.registerArmToAmp(controller.leftTrigger());
        rc.registerArmToHome(controller.rightTrigger());
        rc.registerArmToPodiumShot(controller.x());
        rc.registerArmToFarShot(controller.b());
        rc.registerEnableIntake(controller.a());
        rc.registerDisableIntake(controller.y());
    }

    @Override
    public void registerTeleopFunctions(RobotContainer rc) {
        // none
    }
}
