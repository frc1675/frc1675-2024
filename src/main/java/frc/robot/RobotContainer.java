// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToHome;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.auto.generator.AbstractAutoGenerator;
import frc.robot.auto.generator.PathPlannerAutoGenerator;
import frc.robot.auto.generator.SimpleAutoGenerator;
import frc.robot.cmdGroup.IntakeNote;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.notification.ContextualColor;
import frc.robot.notification.LEDSubsystem;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.SpinUpAndShoot;
import frc.robot.undertaker.UndertakerSubsystem;
import frc.robot.util.DriverDashboard;
import frc.robot.util.RobotContext;
import frc.robot.util.VersionFile;
import frc.robot.util.VoltageDashboard;
import frc.robot.vision.VisionSubsystem;

public class RobotContainer {
  private final PoseScheduler poseScheduler;
  private final DriveSubsystem drive;
  private final ShooterSubsystem shooter;
  private final LEDSubsystem ledSubsystem;
  private final UndertakerSubsystem undertakerSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final ArmSubsystem arm;
  
  private final DriverDashboard dashboard;
  
  private final AbstractAutoGenerator autoGenerator;
  private final RobotContext robotContext;

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Data log started.");

    poseScheduler = new PoseScheduler();
    drive = new DriveSubsystem(poseScheduler);

    visionSubsystem = VisionSubsystem.create();
    ledSubsystem = LEDSubsystem.create();
    undertakerSubsystem = UndertakerSubsystem.create();
    shooter = ShooterSubsystem.create();
    arm = ArmSubsystem.create();

    robotContext = new RobotContext(arm, shooter);

    autoGenerator = 
      Constants.PathPlanner.PATH_PLANNER_IS_ENABLED
      ?
      new PathPlannerAutoGenerator(drive, arm, shooter, undertakerSubsystem, robotContext)
      :
      new SimpleAutoGenerator(drive, shooter, undertakerSubsystem, arm, robotContext);

    //dashboard = new DriverDashboard(robotContext); 
    dashboard = null;

    VoltageDashboard.initVoltageDashboard();
      
    configureBindings();
    VersionFile.getInstance().putToDashboard();
  }

  private void configureBindings() {
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);
    CommandXboxController operatorController = new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER);

    drive.setDefaultCommand(
        new DefaultDrive(drive,
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_Y_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.LEFT_X_AXIS),
            () -> getJoystickInput(driverController, Constants.Controller.RIGHT_X_AXIS),
            robotContext::getDriveSpeedScale
            )
    );

    shooter.setDefaultCommand(new IntakeNote(shooter, undertakerSubsystem, robotContext::getReadyToIntake));
    ledSubsystem.setDefaultCommand(new ContextualColor(robotContext, ledSubsystem, driverController.getHID()));

    driverController.start().onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));
    
    driverController.rightBumper().onTrue(new SpinUpAndShoot(shooter,
     () -> robotContext.getShooterSpeed()[0],
     () -> robotContext.getShooterSpeed()[1]
    ));

    shooter.setDefaultCommand(new IntakeNote(shooter, undertakerSubsystem, robotContext::getReadyToIntake));

    operatorController.leftTrigger().onTrue(new MoveToPosition(arm, Constants.Arm.AMP_POSITION));
    operatorController.rightTrigger().onTrue(new MoveToHome(arm));

    operatorController.x().onTrue(new MoveToPosition(arm, Constants.Arm.LONG_SHOT_ANGLE));
    operatorController.a().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(true)));
    operatorController.y().onTrue(new InstantCommand(() -> robotContext.setIntakeEnabledOverride(false)));
  }

  private double getJoystickInput(CommandGenericHID stick, int axe) {
    return -MathUtil.applyDeadband(stick.getRawAxis(axe), Constants.Controller.DEADZONE_CONSTANT);
  }

  public Command getAutonomousCommand() {
    return autoGenerator.getAutoCommand();
  }

}
