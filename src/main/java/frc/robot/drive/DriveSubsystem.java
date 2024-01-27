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
import swervelib.SwerveModule;
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

    dashboard.add("Velocity Scaler", velocityScale).withWidget("UperTuner").withSize(2, 2).withPosition(5, 1);

    dashboard.add(swerve.field).withPosition(0, 1).withSize(5, 3);
    int position = 0;
    for (SwerveModule m : swerve.getModules()) {
      dashboard.addDouble(m.configuration.name +" Module Position °", () -> m.getAbsolutePosition()).withPosition(position, 0).withSize(2, 1);
      position += 2;
    }
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
    controllerInput[0] = x * Constants.Drive.MAXIMUM_VELOCITY * velocityScale.getCurrentValue();
    controllerInput[1] = y * Constants.Drive.MAXIMUM_VELOCITY * velocityScale.getCurrentValue();
    controllerInput[2] = rotation * Constants.Drive.MAXIMUM_ANGULAR_VELOCITY * velocityScale.getCurrentValue();
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

  /**
   * Used for auto building via path planner
   * 
   * @param override new pose
   */
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
