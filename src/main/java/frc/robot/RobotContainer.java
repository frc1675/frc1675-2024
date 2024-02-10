// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.command.undertaker.EjectNote;
import frc.robot.command.undertaker.IntakeNote;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;
import frc.robot.util.VersionFile;
import frc.robot.vision.IVision;
import frc.robot.vision.RealVision;
import frc.robot.vision.SimVision;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.VisionTestCommand;

public class RobotContainer {
  private final Joystick driverController = new Joystick(Constants.Controller.DRIVER_CONTROLLER);
  private final JoystickButton driverControllerAButton = new JoystickButton(driverController, Constants.Controller.A_BUTTON);
  private final JoystickButton driverControllerBButton = new JoystickButton(driverController, Constants.Controller.B_BUTTON);
  private final DriveSubsystem drive = new DriveSubsystem();
  private final Undertaker undertaker = new Undertaker();
  private final AutoGenerator autoGenerator = new AutoGenerator(drive);
  private final VisionSubsystem visionSubsystem;

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");
    IVision vision;
    if(Robot.isSimulation()){
      vision = new SimVision();
    }else{
      vision = new RealVision();
    }
    visionSubsystem = new VisionSubsystem(vision);
    configureBindings();
    VersionFile.getInstance().putToDashboard();
  }

  private void configureBindings() {
    driverControllerAButton.whileTrue(new IntakeNote(undertaker));
    driverControllerBButton.whileTrue(new EjectNote(undertaker));
    Joystick driverController = new Joystick(Constants.Controller.DRIVER_CONTROLLER);
    JoystickButton driverControllerStartButton = new JoystickButton(driverController, Constants.Controller.START_BUTTON);
    JoystickButton driverControllerLeftStickButton = new JoystickButton(driverController, Constants.Controller.LEFT_JOYSTICK_BUTTON);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS)
        )
    );

    driverControllerStartButton.onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));
    driverControllerLeftStickButton.toggleOnTrue(new VisionTestCommand(visionSubsystem));
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
