package frc.robot.operation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class AbstractCommandXboxOperationConfiguration implements OperationConfiguration {

    protected CommandXboxController controller;

    public AbstractCommandXboxOperationConfiguration(CommandXboxController controller) {
        this.controller = controller;
    }
}
