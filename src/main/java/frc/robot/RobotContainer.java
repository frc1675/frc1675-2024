// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.IArmIO;
import frc.robot.arm.RealArmIO;
import frc.robot.arm.SimArmIO;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.cmdGroup.SpeakerScore;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.undertaker.EjectNote;
import frc.robot.undertaker.IUndertaker;
import frc.robot.undertaker.IntakeNote;
import frc.robot.undertaker.RealUndertaker;
import frc.robot.undertaker.SimUndertaker;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.util.AutoGenerator;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.MathUtils;
import frc.robot.util.VersionFile;
import frc.robot.vision.IVision;
import frc.robot.vision.RealVision;
import frc.robot.vision.SimVision;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.VisionTestCommand;

public class RobotContainer {
  private final PoseScheduler poseScheduler = new PoseScheduler();
  private final DriveSubsystem drive = new DriveSubsystem(poseScheduler);
  private final UndertakerSubsystem undertakerSubsystem;
  private final AutoGenerator autoGenerator = new AutoGenerator(drive);
  private final VisionSubsystem visionSubsystem;
  private final ArmSubsystem arm;

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");

    //poseScheduler.registerCommand(Constants.Field.FRIENDLY_ALLIANCE_AREA, new PrintCommand("I just spun up the motors"));

    drive.setMotorBrakeMode(true);
  
    IArmIO armIO;
    IVision vision;
    IUndertaker undertaker;
    if(Robot.isSimulation()){
      armIO = new SimArmIO();
      vision = new SimVision();
      undertaker = new SimUndertaker();
      armIO = new SimArmIO();
    }else{
      armIO = new RealArmIO();
      vision = new RealVision();
      undertaker = new RealUndertaker();
    }
    arm = new ArmSubsystem(armIO);
    visionSubsystem = new VisionSubsystem(vision);
    undertakerSubsystem = new UndertakerSubsystem(undertaker);

    configureBindings();
    VersionFile.getInstance().putToDashboard();
  }

  private void configureBindings() {
    Joystick operatorController = new Joystick(Constants.Controller.OPERATOR_CONTROLLER);
    JoystickButton operatorAButton = new JoystickButton(operatorController, Constants.Controller.A_BUTTON);
    JoystickButton operatorXButton = new JoystickButton(operatorController, Constants.Controller.X_BUTTON);
    JoystickButton operatorYButton = new JoystickButton(operatorController, Constants.Controller.Y_BUTTON);
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);

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
