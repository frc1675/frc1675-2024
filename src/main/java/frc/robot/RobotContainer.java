// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.cmdGroup.IntakeNote;
import frc.robot.cmdGroup.SpinUpAndShoot;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.notification.LEDSubsystem;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.AutoGenerator;
import frc.robot.util.MathUtils;
import frc.robot.util.VersionFile;
import frc.robot.vision.VisionSubsystem;

public class RobotContainer {
  private final PoseScheduler poseScheduler;
  private final DriveSubsystem drive;
  private final ShooterSubsystem shooter;
  private final LEDSubsystem ledSubsystem;
  private final UndertakerSubsystem undertakerSubsystem;
  private final AutoGenerator autoGenerator;
  private final VisionSubsystem visionSubsystem;
  private final ArmSubsystem arm;

  private boolean intakeEnabled = true;

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");

    poseScheduler = new PoseScheduler();
    drive = new DriveSubsystem(poseScheduler);
    drive.setMotorBrakeMode(true);
    autoGenerator = new AutoGenerator(drive);

    arm = ArmSubsystem.create();
    visionSubsystem = VisionSubsystem.create();
    ledSubsystem = LEDSubsystem.create();
    undertakerSubsystem = UndertakerSubsystem.create();
    shooter = ShooterSubsystem.create();

    configureBindings();
    VersionFile.getInstance().putToDashboard();
  }

  private void configureBindings() {
    CommandXboxController operatorController = new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER);
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS),
            () -> getDriveSpeedScale()

            )
    );

    shooter.setDefaultCommand(new IntakeNote(shooter, undertakerSubsystem, () -> arm.isAtHomePostion() && intakeEnabled));

    driverController.start().onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));
    driverController.rightTrigger().onTrue(new SpinUpAndShoot(shooter, undertakerSubsystem, () -> arm.isAtAmpPosition()));

    operatorController.leftTrigger().onTrue(new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
    operatorController.rightTrigger().onTrue(new MoveToHome(arm));

    operatorController.a().onTrue(new InstantCommand(() -> intakeEnabled = true));
    operatorController.y().onTrue(new InstantCommand(() -> intakeEnabled = false));
  }

  private double getJoystickInput(CommandXboxController stick, int axe) {
    return -MathUtils.getDeadzoneAdjustedInput(stick.getRawAxis(axe));
  }

  private double getDriveSpeedScale() {
    return arm.getAngle() <= Constants.Arm.HIGH_SCORE_POSITION ? Constants.Drive.SLOW_DRIVE_SCALE : 1;
  }

  public Command getAutonomousCommand() {
    return autoGenerator.getAutoCommand();
  }

  public void updateFieldMap() {
    autoGenerator.updateMap();
  }

  public void onDisabled() {
    // Timer.delay(10); // Wait so that any momentum from the match is absorbed by
    // the brakes before
    // setting to coast.
    // if (drive != null)
    // drive.setMotorBrakeMode(false);
  }

}
