// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.notification.ChangeColor;
import frc.robot.notification.LEDSubsystem;
import frc.robot.notification.RealLedIO;
import frc.robot.notification.ILedIO;
import frc.robot.notification.SimLedIO;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.undertaker.IUndertaker;
import frc.robot.undertaker.RealUndertaker;
import frc.robot.undertaker.SimUndertaker;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;
import frc.robot.util.VersionFile;
import frc.robot.vision.IVision;
import frc.robot.vision.RealVision;
import frc.robot.vision.SimVision;
import frc.robot.vision.VisionSubsystem;

public class RobotContainer {
  private final PoseScheduler poseScheduler = new PoseScheduler();
  private final DriveSubsystem drive = new DriveSubsystem(poseScheduler);
  private final LEDSubsystem ledSubsystem;
  private final UndertakerSubsystem undertakerSubsystem;
  private final AutoGenerator autoGenerator = new AutoGenerator(drive);
  private final VisionSubsystem visionSubsystem;

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");

    //poseScheduler.registerCommand(Constants.Field.FRIENDLY_ALLIANCE_AREA, new PrintCommand("I just spun up the motors"));

    drive.setMotorBrakeMode(true);
  
    ILedIO ledIO; 
    IVision vision;
    IUndertaker undertaker;
    if(Robot.isSimulation()){
      vision = new SimVision();
      ledIO = new SimLedIO(); 
      undertaker = new SimUndertaker();
    }else{
      vision = new RealVision();
      ledIO = new RealLedIO();
      undertaker = new RealUndertaker();
    }
  
    visionSubsystem = new VisionSubsystem(vision);
    ledSubsystem = new LEDSubsystem(ledIO); 
    undertakerSubsystem = new UndertakerSubsystem(undertaker);

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

    driverController.a().onTrue(new ChangeColor(ledSubsystem));
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
