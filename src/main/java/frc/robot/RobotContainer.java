// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.arm.Arm;
import frc.robot.arm.IArmIO;
import frc.robot.arm.RealArmIO;
import frc.robot.arm.SimArmIO;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;

public class RobotContainer {

  private DriveSubsystem drive = new DriveSubsystem();
  private Arm arm;
  private AutoGenerator autoGenerator = new AutoGenerator(drive);

  public RobotContainer() {
    IArmIO armIO;
    if (Robot.isSimulation()) {
      armIO = new SimArmIO();
    } else {
      armIO = new RealArmIO();
    }

    arm = new Arm(armIO);

    configureBindings();
  }

  private void configureBindings() {
    Joystick driverController = new Joystick(Constants.Controller.DRIVER_CONTROLLER);
    Joystick operatorController = new Joystick(Constants.Controller.OPERATOR_CONTROLLER);
    JoystickButton operatorAButton = new JoystickButton(operatorController, Constants.Controller.A_BUTTON);
    JoystickButton operatorXButton = new JoystickButton(operatorController, Constants.Controller.X_BUTTON);
    JoystickButton operatorYButton = new JoystickButton(operatorController, Constants.Controller.Y_BUTTON);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS)
        )
    );

    operatorAButton.onTrue(
      new MoveToPosition(arm, Constants.Arm.LOW_SCORE_POSITION)
    );

    operatorYButton.onTrue(
      new MoveToPosition(arm, Constants.Arm.HIGH_SCORE_POSITION)
    );

    operatorXButton.onTrue(
      new MoveToHome(arm)
    );
  }

  private double getJoystickInput(Joystick stick, int axe) {
    return -MathUtils.getDeadzoneAdjustedInput(stick.getRawAxis(axe));
  }

  public Command getAutonomousCommand() {
    return autoGenerator.getAutoCommand();
  }

  public void updateFieldMap() {
    autoGenerator.updateMap();
  }

}
