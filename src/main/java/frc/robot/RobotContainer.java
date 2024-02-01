// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MoveToHome;
import frc.robot.commands.MoveToPosition;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;

public class RobotContainer {

  private DriveSubsystem drive = new DriveSubsystem();
  private Arm arm = new Arm();
  private AutoGenerator autoGenerator = new AutoGenerator(drive);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Joystick driverController = new Joystick(Constants.Controller.DRIVER_CONTROLLER);
    Joystick operatorController = new Joystick(Constants.Controller.OPERATOR_CONTROLLER);
    JoystickButton operatorAButton = new JoystickButton(operatorController, Constants.Controller.A_BUTTON);
    JoystickButton operatorBButton = new JoystickButton(operatorController, Constants.Controller.B_BUTTON);

    JoystickButton driverAButton = new JoystickButton(driverController, Constants.Controller.A_BUTTON);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS)
        )
    );

    operatorAButton.onTrue(
      new MoveToPosition(arm, Constants.Arm.HIGH_SCORE_POSITION)
    );

    operatorBButton.onTrue(
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
