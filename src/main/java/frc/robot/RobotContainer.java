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
import frc.robot.undertaker.IUndertaker;
import frc.robot.undertaker.IntakeNote;
import frc.robot.undertaker.RealUndertaker;
import frc.robot.undertaker.SimUndertaker;
import frc.robot.Constants.Undertaker;
import frc.robot.cmdGroup.NoteToShooter;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.util.AutoGenerator;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.MathUtils;
import frc.robot.shooter.*;
import frc.robot.shooter.commands.Shoot;
import frc.robot.shooter.commands.SpinUp;
import frc.robot.shooter.commands.SpinUpAndShoot;
import frc.robot.util.VersionFile;
import frc.robot.vision.IVision;
import frc.robot.vision.RealVision;
import frc.robot.vision.SimVision;
import frc.robot.vision.VisionSubsystem;

public class RobotContainer {
  private final PoseScheduler poseScheduler = new PoseScheduler();
  private final DriveSubsystem drive = new DriveSubsystem(poseScheduler);
  private ShooterSubsystem shooter;
  private final UndertakerSubsystem undertakerSubsystem;
  private final AutoGenerator autoGenerator = new AutoGenerator(drive);
  private final VisionSubsystem visionSubsystem;

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");

    //poseScheduler.registerCommand(Constants.Field.FRIENDLY_ALLIANCE_AREA, new PrintCommand("I just spun up the motors"));

    drive.setMotorBrakeMode(true);
  
    IVision vision;
    IUndertaker undertaker;
    if(Robot.isSimulation()){
      vision = new SimVision();
      undertaker = new SimUndertaker();
    }else{
      vision = new RealVision();
      undertaker = new RealUndertaker();
    }
  
    visionSubsystem = new VisionSubsystem(vision);
    undertakerSubsystem = new UndertakerSubsystem(undertaker);

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");

    IShooterIO shooterIO;
    if (Robot.isSimulation()) {
        shooterIO = new SimShooterIO();
    } else {
        shooterIO = new RealShooterIO();
    }

    shooter = new ShooterSubsystem(shooterIO);
       
    configureBindings();
    VersionFile.getInstance().putToDashboard();
  }

  private void configureBindings() {
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);
    CommandXboxController operatorController  = new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS)
        )
    );

    driverController.start().onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));

    operatorController.x().onTrue(new NoteToShooter(shooter, undertakerSubsystem));

    // shoot (indexer)
    operatorController.y().whileTrue(new Shoot(shooter, 1)); // run indexer at 100% to shoot

    // spinup and hold speed
    operatorController.a().onTrue(new SpinUp(shooter));

    // operatorController.b().onTrue(new SpinUpAndShoot(shooter));

    // operatorController.x().whileTrue(new InstantCommand(() -> {
    //     shooter.setTargetShooterSpeed(Constants.Shooter.TARGET_SHOOTER_SPEED);
    // }));

    // operatorController.x().onFalse(new InstantCommand(() -> {
    //     shooter.setTargetShooterSpeed(0);
    // }));
  }

  private double getJoystickInput(CommandXboxController stick, int axe) {
    return -MathUtils.getDeadzoneAdjustedInput(stick.getRawAxis(axe));
  }

  public Command getAutonomousCommand() {
    return null; //autoGenerator.getAutoCommand();
  }

  public void updateFieldMap() {
    //autoGenerator.updateMap();
  } 

  public void onDisabled() {
    Timer.delay(10); // Wait so that any momentum from the match is absorbed by the brakes before setting to coast. 

    drive.setMotorBrakeMode(false);
  }
}
