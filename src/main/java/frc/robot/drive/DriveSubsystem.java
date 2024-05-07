// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.poseScheduler.PoseScheduler;
import frc.robot.util.UperTunerSendable;

import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class DriveSubsystem extends SubsystemBase {

    private SwerveDrive swerve;

    private final PoseScheduler poseScheduler;

    private ShuffleboardTab dashboard;

    private final Field2d m1 = new Field2d();
    private final Field2d m2 = new Field2d();


    private PoseScheduler posScheduler;
    private UperTunerSendable headingTuneableP;
    private UperTunerSendable headingTuneableI;
    private UperTunerSendable headingTuneableD;

    private PIDFConfig headingPidfConfig;

    private final PIDController rotationController;
    private Double targetAngle = null;
    private int rotationDirection =
            1; // Used to make sure the rotation PID continues working while the gyroscope is inverted.

    public DriveSubsystem(PoseScheduler poseScheduler) {
        this.poseScheduler = poseScheduler;
        rotationController =
                new PIDController(Constants.Drive.ROTATION_P, Constants.Drive.ROTATION_I, Constants.Drive.ROTATION_D);
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(Constants.Drive.ROTATION_TARGET_RANGE);

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(
                            Constants.Drive.MAXIMUM_VELOCITY,
                            SwerveMath.calculateDegreesPerSteeringRotation(
                                    Constants.Drive.STEER_GEAR_RATIO, Constants.Drive.PULSE_PER_ROTATION),
                            SwerveMath.calculateMetersPerRotation(
                                    Constants.Drive.WHEEL_DIAMETER_METERS,
                                    Constants.Drive.DRIVE_GEAR_RATIO,
                                    Constants.Drive.PULSE_PER_ROTATION));
        } catch (IOException e) {
            System.out.println("Swerve drive configuration file could not be found at "
                    + Filesystem.getDeployDirectory()
                    + "/swerve");
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
        dashboard.addDouble("Yaw", () -> swerve.getYaw().getDegrees());
        dashboard.addDouble("Target angle", () -> targetAngle == null ? -1 : targetAngle);

        dashboard.addBoolean("Rotation On Target?", () -> rotationController.atSetpoint());

        dashboard.add(swerve.field).withPosition(0, 1).withSize(5, 3);
        int position = 0;
        for (SwerveModule m : swerve.getModules()) {
            dashboard
                    .addDouble(m.configuration.name + " Module Position °", () -> m.getAbsolutePosition())
                    .withPosition(position, 0)
                    .withSize(2, 1);
            position += 2;
        }
        
        dashboard.add(swerve.field).withPosition(0, 1).withSize(5, 3);
        dashboard.add("Vision Pose", m1);
        dashboard.add("Vision Pose 2", m2);
    }

    private String getCommandName() {
        if (this.getCurrentCommand() == null) {
            return "None";
        }
        return this.getCurrentCommand().getName();
    }

    /**
     * Zero the gyroscope. This is useful for resetting which way is considered positive for field
     * relative robot driving. This should probably only be done while debugging.
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

    /** Used for PathPlanner autonomous */
    public void setRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        swerve.drive(
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond,
                false,
                false);
    }

    public void drive(double x, double y, double rotation) {
        if (rotation != 0 || (targetAngle != null && rotationController.atSetpoint())) {
            targetAngle = null;
        }

        if (targetAngle != null) {
            rotation = rotationDirection
                    * rotationController.calculate(swerve.getYaw().getDegrees(), targetAngle);
        } else {
            rotation = rotation * Constants.Drive.MAXIMUM_ANGULAR_VELOCITY;
        }

        swerve.drive(
                new Translation2d(x * Constants.Drive.MAXIMUM_VELOCITY, y * Constants.Drive.MAXIMUM_VELOCITY),
                rotation,
                true,
                false);
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public void addVisionMeasurement(Pose2d visionMeasuredPose) {
        // Per recommendation from lib authors, discard poses which are
        // too far away from current pose.
        double distance =
                Math.sqrt(Math.pow((visionMeasuredPose.getX() - swerve.getPose().getX()), 2)
                        + Math.pow((visionMeasuredPose.getY() - swerve.getPose().getY()), 2));
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
        
    if (headingPidfConfig != null) {
        headingPidfConfig.p = headingTuneableP.getCurrentValue();
        headingPidfConfig.i = headingTuneableI.getCurrentValue();
        headingPidfConfig.d = headingTuneableD.getCurrentValue();
    }
    LimelightHelpers.SetRobotOrientation(Constants.Drive.LIMELIGHT_NAME, swerve.getYaw().getDegrees(), 0, 0, 0, 0, 0);
    m1.setRobotPose(LimelightHelpers.getBotPose2d_wpiBlue(Constants.Drive.LIMELIGHT_NAME));
    m2.setRobotPose(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Drive.LIMELIGHT_NAME).pose);
}
}