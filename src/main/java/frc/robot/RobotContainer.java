// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;
import frc.robot.shooter.*;

public class RobotContainer {

  private DriveSubsystem drive = new DriveSubsystem();
  private ShooterSubsystem shooter = new ShooterSubsystem();
  private AutoGenerator autoGenerator = new AutoGenerator(drive);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Joystick driverController = new Joystick(Constants.Controller.DRIVER_CONTROLLER);
    Joystick operatorController = new Joystick(Constants.Controller.OPERATOR_CONTROLLER);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS)
        )
    );


    JoystickButton xButton = new JoystickButton(operatorController, Constants.Controller.X_BUTTON);
    JoystickButton yButton = new JoystickButton(operatorController, Constants.Controller.Y_BUTTON);

    xButton.toggleOnTrue(new SpinUpAndShoot(shooter));
    
    // move to undertaker subsystem:
    yButton.toggleOnTrue(new ShooterIntake(shooter, 200));


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
