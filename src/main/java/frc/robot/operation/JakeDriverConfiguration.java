package frc.robot.operation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceUtil;

public class JakeDriverConfiguration extends AbstractCommandXboxOperationConfiguration {

    public JakeDriverConfiguration(CommandXboxController controller) {
        super(controller);
    }

    @Override
    public void registerRobotFunctions(RobotContainer rc) {
        rc.registerZeroGyro(controller.start());
        rc.registerShootAndGoHome(controller.rightBumper());
        rc.registerTurnToSpeakerWall(controller.a());
        rc.registerAimFromPodium(controller.b());
        rc.registerTurnToAmp(controller.x());
    }

    @Override
    public void registerTeleopFunctions(RobotContainer rc) {
        rc.registerDefaultDrive(
                () -> AllianceUtil.getTranslationDirection()
                        * getJoystickInput(controller, Constants.Controller.LEFT_Y_AXIS),
                () -> AllianceUtil.getTranslationDirection()
                        * getJoystickInput(controller, Constants.Controller.LEFT_X_AXIS),
                () -> getJoystickInput(controller, Constants.Controller.RIGHT_X_AXIS));
    }

    private double getJoystickInput(CommandGenericHID stick, int axe) {
        return MathUtil.applyDeadband(stick.getRawAxis(axe), Constants.Controller.DEADZONE_CONSTANT);
    }
}
