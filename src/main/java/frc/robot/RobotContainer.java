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
import frc.robot.notification.ChangeColor;
import frc.robot.notification.LEDSubsystem;
import frc.robot.notification.RealLedIO;
import frc.robot.notification.ILedIO;
import frc.robot.notification.SimLedIO;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;
import frc.robot.util.VersionFile;
import frc.robot.vision.IVision;
import frc.robot.vision.RealVision;
import frc.robot.vision.SimVision;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.VisionTestCommand;

public class RobotContainer {

  private final DriveSubsystem drive = new DriveSubsystem();
  private final LEDSubsystem ledSubsystem;
  private final AutoGenerator autoGenerator = new AutoGenerator(drive);
  private final Joystick driverController = new Joystick(Constants.Controller.DRIVER_CONTROLLER);
  private final JoystickButton driverControllerAButton = new JoystickButton(driverController, Constants.Controller.A_BUTTON);
  private final JoystickButton driverControllerStartButton = new JoystickButton(driverController, Constants.Controller.START_BUTTON);
  private final JoystickButton driverControllerLeftStickButton = new JoystickButton(driverController, Constants.Controller.LEFT_JOYSTICK_BUTTON);
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
     ILedIO ledIO; 
    if(Robot.isSimulation()){
      ledIO = new SimLedIO(); 
    }else{
      ledIO = new RealLedIO();
    }
    ledSubsystem = new LEDSubsystem(ledIO); 
    configureBindings();
    VersionFile.getInstance().putToDashboard();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS)
        )
    );

    driverControllerAButton.onTrue(new ChangeColor(ledSubsystem));
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
