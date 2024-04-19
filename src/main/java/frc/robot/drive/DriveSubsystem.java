// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

  private final SwerveDrive swerve;

  private final PIDController rotationController = new PIDController(Constants.Drive.ROTATION_P, Constants.Drive.ROTATION_I, Constants.Drive.ROTATION_D);
  private Rotation2d rotationTarget = null;

  private ChassisSpeeds inputSpeeds = new ChassisSpeeds();
  private boolean isFieldRelative = true;


  public DriveSubsystem() {
    rotationController.enableContinuousInput(-180, 180);
    rotationController.setTolerance(Constants.Drive.ROTATION_TARGET_RANGE);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(
        Constants.Drive.MAXIMUM_VELOCITY, 
        SwerveMath.calculateDegreesPerSteeringRotation(Constants.Drive.STEER_GEAR_RATIO, Constants.Drive.PULSE_PER_ROTATION), 
        SwerveMath.calculateMetersPerRotation(Constants.Drive.WHEEL_DIAMETER_METERS, Constants.Drive.DRIVE_GEAR_RATIO, Constants.Drive.PULSE_PER_ROTATION)
      );
    } catch (IOException e) {
      e.printStackTrace();
      throw new Error("Swerve drive configuration could not be loaded from " + Filesystem.getDeployDirectory() + "/swerve", e);
    }

    swerve.chassisVelocityCorrection = false; //TODO test out setting this
    swerve.setHeadingCorrection(true);
    initDashboard();
  }

  private void initDashboard() {
    ShuffleboardTab dashboard = Shuffleboard.getTab("Drive");
    dashboard.addString("Current Command", this::getCommandName);

    dashboard.add("Rotation PID", rotationController);
    dashboard.addDouble("Yaw", () ->swerve.getYaw().getDegrees());
    dashboard.addDouble("Target angle", () -> rotationTarget == null ? -1 : rotationTarget.getDegrees());
    dashboard.addBoolean("Has rotation target?", () -> rotationTarget == null);

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
   * 
   * @param override new pose
   */
  public void resetOdometry(Pose2d override) {
    swerve.resetOdometry(override);
  }

  /**
   * Used for PathPlanner autonomous
   * 
   * @param speeds robot relative chassis speeds
   */
  public void setRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
    inputSpeeds = speeds;
    isFieldRelative = false;
  }

  /**
   * Get the current robot pose as measured by the odometry. 
   * Odometry may include vision based measurements.
   * @return the current pose of the robot
   */
  public Pose2d getPose() {
    return swerve.getPose();
  }

  /**
   * Drive the robot in field relative mode. 
   * This is the primary method of teleoperated robot control.
   * 
   * @param x percent translational speed in the x direction
   * @param y percent translational speed in the y direction
   * @param rotation percent rotation speed
   */
  public void drive(double x, double y, double rotation) {
    inputSpeeds = new ChassisSpeeds(
      x * Constants.Drive.MAXIMUM_VELOCITY,
      y * Constants.Drive.MAXIMUM_VELOCITY,
      rotation * Constants.Drive.MAXIMUM_ANGULAR_VELOCITY
    );

    isFieldRelative = true;
  }

  /**
   * Add a vision measurement to the pose estimate. 
   * @param visionMeasuredPose The pose the vision measured.
   * @param measurementTimeSeconds The time the given pose was measured in seconds
   */
  public void addVisionMeasurement(Pose2d visionMeasuredPose, double measurementTimeSeconds) {
    // Per recommendation from lib authors, discard poses which are 
    // too far away from current pose.
    double distance = Math.sqrt(
        Math.pow((visionMeasuredPose.getX() - swerve.getPose().getX()), 2)
        +
        Math.pow((visionMeasuredPose.getY() - swerve.getPose().getY()), 2)
      );
    if (distance <= Constants.Drive.MAXIMUM_VISION_POSE_OVERRIDE_DISTANCE) {
      swerve.swerveDrivePoseEstimator.addVisionMeasurement(visionMeasuredPose, measurementTimeSeconds);
    }

  }

  /**
   * Set a rotation target. May be null.
   * @param target Rotation target or null to disable target
   */
  public void setRotationTarget(Rotation2d target) {
    rotationTarget = target;
  }

  @Override
  public void periodic() {
    if (inputSpeeds.omegaRadiansPerSecond != 0) {
      rotationTarget = null;
    }else if (rotationTarget != null) {
      inputSpeeds.omegaRadiansPerSecond = rotationController.calculate(
        MathUtil.inputModulus(swerve.getYaw().getDegrees(), -180, 180), 
        MathUtil.inputModulus(rotationTarget.getDegrees(), -180, 180)
      );
    }

    if (isFieldRelative) {
      swerve.driveFieldOriented(inputSpeeds);
    }else {
      swerve.drive(inputSpeeds);
    }
  }
}
