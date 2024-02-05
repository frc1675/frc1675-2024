// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;
import frc.robot.util.VersionFile;

public class RobotContainer {

  private DriveSubsystem drive = new DriveSubsystem();
  private AutoGenerator autoGenerator = new AutoGenerator(drive);

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");

    configureBindings();
    VersionFile.getInstance().putToDashboard();
  }

  private void configureBindings() {
    Joystick driverController = new Joystick(Constants.Controller.DRIVER_CONTROLLER);
    JoystickButton driverControllerStartButton = new JoystickButton(driverController, Constants.Controller.START_BUTTON);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS)
        )
    );

    driverControllerStartButton.onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));
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
