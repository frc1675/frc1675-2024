// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.poseScheduler.PoseScheduler;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

  private SwerveDrive swerve;

  private final PoseScheduler poseScheduler;

  private ShuffleboardTab dashboard;

  private final PIDController rotationController;
  private Double targetAngle = null;
  private int rotationDirection = 1; //Used to make sure the rotation PID continues working while the gyroscope is inverted.


  public DriveSubsystem(PoseScheduler poseScheduler) {
    this.poseScheduler = poseScheduler;
    rotationController = new PIDController(Constants.Drive.ROTATION_P, Constants.Drive.ROTATION_I, Constants.Drive.ROTATION_D);
    rotationController.enableContinuousInput(-180, 180);
    rotationController.setTolerance(Constants.Drive.ROTATION_TARGET_RANGE);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(
        Constants.Drive.MAXIMUM_VELOCITY, 
        SwerveMath.calculateDegreesPerSteeringRotation(Constants.Drive.STEER_GEAR_RATIO, Constants.Drive.PULSE_PER_ROTATION), 
        SwerveMath.calculateMetersPerRotation( Constants.Drive.WHEEL_DIAMETER_METERS, Constants.Drive.DRIVE_GEAR_RATIO, Constants.Drive.PULSE_PER_ROTATION)
      );
    } catch (IOException e) {
      System.out.println("Swerve drive configuration file could not be found at " + Filesystem.getDeployDirectory() + "/swerve");
      e.printStackTrace();
    }

    swerve.chassisVelocityCorrection = false;    
    swerve.setHeadingCorrection(true);
    initDashboard();
  }

  private void initDashboard() {
    dashboard = Shuffleboard.getTab("Drive");
    dashboard.addString("Current Command", this::getCommandName);

    dashboard.add("Rotation PID", rotationController);
    dashboard.addDouble("Yaw", () ->swerve.getYaw().getDegrees());
    dashboard.addDouble("Target angle", () -> targetAngle == null ? -1 : targetAngle);

    dashboard.addBoolean("Rotation On Target?", () -> rotationController.atSetpoint());


    dashboard.add(swerve.field).withPosition(0, 1).withSize(5, 3);
    int position = 0;
    for (SwerveModule m : swerve.getModules()) {
      dashboard.addDouble(m.configuration.name +" Module Position °", () -> m.getAbsolutePosition()).withPosition(position, 0).withSize(2, 1);
      position += 2;
    }
  }

  private String getCommandName() {
    if(this.getCurrentCommand() == null) {
      return "None";
    }
    return this.getCurrentCommand().getName();
  }

  /**
   * Zero the gyroscope. This is useful for resetting which way is considered
   * positive for field relative robot driving. This should probably only be done
   * while debugging.
   */
  public void zeroGyroscope() {
    swerve.zeroGyro();
  }

  public void zeroGyroscope(double degreesOffset) {
    Rotation3d current = this.swerve.getGyroRotation3d();

    swerve.setGyro(new Rotation3d(current.getX(), current.getY(), Units.degreesToRadians(degreesOffset)));
  }

  /**
   * Used for PathPlanner autonomous
   * 
   * @return robot relative chassis speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerve.getRobotVelocity();
  }

  /**
   * Used for PathPlanner autonomous
   */
  public void setRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
    swerve.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, false);
  }

  /**
   * Used for simple autonomous
   * Drive the drivetrain in a single direction
   * @param velocity The desired velocity. Will be capped at the max velocity
   * @param isX True if the drivetrain should drive in the x direction, false otherwise
   */
  public void singleDirectionDrive(double velocity, boolean isX) {
    velocity = Math.min(velocity, Constants.Drive.MAXIMUM_VELOCITY);

    if (isX) {
      swerve.drive(new Translation2d(velocity, 0), 0, true, false);
    }else  {
      swerve.drive(new Translation2d(0, velocity), 0, true, false);
    }
  }

  public void drive(double x, double y, double rotation) {
    if (rotation != 0 || (targetAngle != null && rotationController.atSetpoint())) {
      targetAngle = null;
    }

    if (targetAngle != null) {
      rotation = rotationDirection * rotationController.calculate(swerve.getYaw().getDegrees(), targetAngle);
    }else {
      rotation = rotation * Constants.Drive.MAXIMUM_ANGULAR_VELOCITY;
    }

    swerve.drive(
        new Translation2d(
          x * Constants.Drive.MAXIMUM_VELOCITY, 
          y * Constants.Drive.MAXIMUM_VELOCITY
        ),
        rotation,
        true, false
      ); 
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  public void addVisionMeasurement(Pose2d visionMeasuredPose) {
    // Per recommendation from lib authors, discard poses which are 
    // too far away from current pose.
    double distance = Math.sqrt(
        Math.pow((visionMeasuredPose.getX() - swerve.getPose().getX()), 2)
        +
        Math.pow((visionMeasuredPose.getY() - swerve.getPose().getY()), 2)
      );
    if (distance <= Constants.Drive.MAXIMUM_VISION_POSE_OVERRIDE_DISTANCE) {
      swerve.swerveDrivePoseEstimator.addVisionMeasurement(visionMeasuredPose, Timer.getFPGATimestamp());
    }

  }

  /**
   * Used for autonomous
   * 
   * @param override new pose
   */
  public void resetOdometry(Pose2d override) {
    swerve.resetOdometry(override);
  }

  public void setTargetAngle(double angleDeg) {
    rotationController.reset();
    targetAngle = angleDeg;
  }

  public void unsetTargetAngle() {
    targetAngle = null;
  }

  @Override
  public void periodic() {
    poseScheduler.updatePose(getPose());
  }
}
