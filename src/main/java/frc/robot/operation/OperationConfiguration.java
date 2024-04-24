package frc.robot.operation;

import frc.robot.RobotContainer;

/**
 * An operation configuration is an object that can register Triggers and DoubleSuppliers to robot functions.
 */
public interface OperationConfiguration {

    public void registerRobotFunctions(RobotContainer rc);

    public void registerTeleopFunctions(RobotContainer rc);
}
