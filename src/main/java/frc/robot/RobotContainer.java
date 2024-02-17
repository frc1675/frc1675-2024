// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.cmdGroup.SpeakerScore;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;
import frc.robot.util.VersionFile;

public class RobotContainer {

  private PoseScheduler poseScheduler = new PoseScheduler();
  private DriveSubsystem drive = new DriveSubsystem(poseScheduler);
  private AutoGenerator autoGenerator = new AutoGenerator(drive);

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");

    //poseScheduler.registerCommand(Constants.Field.FRIENDLY_ALLIANCE_AREA, new PrintCommand("I just spun up the motors"));

    drive.setMotorBrakeMode(true);

    configureBindings();
    VersionFile.getInstance().putToDashboard();
  }

  private void configureBindings() {
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS)
        )
    );

    driverController.start().onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));
    
    //driverController.a().onTrue(new SpeakerScore(drive, autoGenerator));
  }

  private double getJoystickInput(CommandXboxController stick, int axe) {
    return -MathUtils.getDeadzoneAdjustedInput(stick.getRawAxis(axe));
  }

  public Command getAutonomousCommand() {
    return autoGenerator.getAutoCommand();
  }

  public void updateFieldMap() {
    autoGenerator.updateMap();
  }

  public void onDisabled() {
    Timer.delay(10); // Wait so that any momentum from the match is absorbed by the brakes before setting to coast. 

    drive.setMotorBrakeMode(false);
  }

}
