// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.UperTunerSendable;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class DriveSubsystem extends SubsystemBase {

  private SwerveDrive swerve;
  private double[] controllerInput = { 0, 0, 0 }; // [x, y, rotation]

  private UperTunerSendable velocityScale = new UperTunerSendable(1.0, 0.0, 1.0);

  private ShuffleboardTab dashboard;

  public DriveSubsystem() {
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

    initDashboard();
    swerve.chassisVelocityCorrection = false;
  }

  private void initDashboard() {
    dashboard = Shuffleboard.getTab("Drive");
    SendableRegistry.add(new UperTunerSendable(0), "UperTuner");

    dashboard.add("Velocity Scaler", velocityScale).withWidget("UperTuner").withSize(2, 2);

    dashboard.add(swerve.field);
    dashboard.addDouble("Module 1 Position", () -> swerve.getModules()[0].getAbsolutePosition()).withPosition(0, 0);
    dashboard.addDouble("Module 2 Position", () -> swerve.getModules()[1].getAbsolutePosition()).withPosition(1, 0);
    dashboard.addDouble("Module 3 Position", () -> swerve.getModules()[2].getAbsolutePosition()).withPosition(0, 1);
    dashboard.addDouble("Module 4 Position", () -> swerve.getModules()[3].getAbsolutePosition()).withPosition(1, 1);
  }

  /**
   * Used for auto building via path planner
   * 
   * @return robot relative chassis speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerve.getRobotVelocity();
  }

  /**
   * Used for auto building via path planner
   */
  public void setRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }

  public void drive(double x, double y, double rotation) {
    controllerInput[0] = x * Constants.Drive.MAXIMUM_VELOCITY;
    controllerInput[1] = y * Constants.Drive.MAXIMUM_VELOCITY;
    controllerInput[2] = rotation * Constants.Drive.MAXIMUM_ANGULAR_VELOCITY;
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
    if (distance <= Constants.Drive.MAXIMUM_VISON_POSE_OVERRIDE_DISTANCE) {
      swerve.swerveDrivePoseEstimator.addVisionMeasurement(visionMeasuredPose, Timer.getFPGATimestamp());
    }

  }

  public void resetOdometry(Pose2d override) {
    swerve.resetOdometry(override);
  }

  @Override
  public void periodic() {
    swerve.drive(
        new Translation2d(controllerInput[0], controllerInput[1]),
        controllerInput[2],
        true, false
      );

    swerve.updateOdometry();
  }
}
