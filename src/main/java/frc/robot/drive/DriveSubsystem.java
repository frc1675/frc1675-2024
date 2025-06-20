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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    // JTPTODO REMOVE initdashboard for brownbox version
    private ShuffleboardTab dashboard;

    private SwerveDrive swerve;

    private final PIDController rotationController;
    private Double targetAngle = null;

    private double maxTranslationVelocity;
    private double maxRotationVelocity;
    private double maxVisionPoseOverrideDistance;

    /*
     * Josh concerns:
     * - rotation controller feels messy but unsure if there is a better way to handle the desired action
     *   (snap to commanded angles but be controlled by stick otherwise)
     */

    /**
     * Constructs a field-relative swerve drive subsystem that uses YAGSL and has needed methods for use with PathPlanner.
     * Additionally provides capability for automatically pointing to a specific heading.
     * CAN IDs for drive motors; turning motors, and turning sensors are configured in the YAGSL json files at deploy/swerve.
     *
     * @param maxTranslationVelocity Maximum translation velocity of the robot in meters per second
     * @param maxRotationVelocity Maximum rotation velocity of the robot in radians per second
     * @param steeringGearRatio Steering gear ratio (for reference, 12.8 in 2024)
     * @param steeringPulsePerRotation PPR of the steering motor (1 if integrated, 1 in 2024)
     * @param driveGearRatio Drive gear ratio (for reference, 6.12 in 2024)
     * @param drivePulsePerRotation PPR of the drive motor (1 if integrated, 1 in 2024)
     * @param wheelDiameter Wheel diameter in meters
     * @param pointingP P for automatic pointing rotation
     * @param pointingI I for automatic pointing rotation
     * @param pointingD D for automatic pointing rotation
     * @param pointingTolerance Tolerance for automatic pointing, in degrees.
     * @param maxVisionPoseOverrideDistance Maximum vision pose difference allowed, in meters. (If the vision pose is too far off, ignore it)
     */
    private DriveSubsystem(DriveSubsystemBuilder builder) {

        this.maxRotationVelocity = builder.maxRotationVelocity;
        this.maxTranslationVelocity = builder.maxTranslationVelocity;
        this.maxVisionPoseOverrideDistance = builder.maxVisionPoseOverrideDistance;

        rotationController = new PIDController(builder.pointingP, builder.pointingI, builder.pointingD);
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(builder.pointingTolerance);

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(
                            maxTranslationVelocity,
                            SwerveMath.calculateDegreesPerSteeringRotation(
                                    builder.steeringGearRatio, builder.steeringPulsePerRotation),
                            SwerveMath.calculateMetersPerRotation(
                                    builder.wheelDiameter, builder.driveGearRatio, builder.drivePulsePerRotation));
        } catch (IOException e) {
            System.out.println("Swerve drive configuration file could not be found at "
                    + Filesystem.getDeployDirectory()
                    + "/swerve");
            e.printStackTrace();
        }

        swerve.chassisVelocityCorrection = false;
        swerve.setHeadingCorrection(true);

        // JTPTODO REMOVE initdashboard for brownbox version
        initDashboard();
    }

    // JTPTODO REMOVE initdashboard for brownbox version
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

    public void drive(double x, double y, double rotation) {
        if (rotation != 0 || (targetAngle != null && rotationController.atSetpoint())) {
            targetAngle = null;
        }

        if (targetAngle != null) {
            rotation = rotationController.calculate(swerve.getYaw().getDegrees(), targetAngle);
        } else {
            rotation = rotation * maxRotationVelocity;
        }

        swerve.drive(new Translation2d(x * maxTranslationVelocity, y * maxTranslationVelocity), rotation, true, false);
    }

    public void addVisionMeasurement(Pose2d visionMeasuredPose) {
        // Per recommendation from lib authors, discard poses which are
        // too far away from current pose.
        double distance =
                Math.sqrt(Math.pow((visionMeasuredPose.getX() - swerve.getPose().getX()), 2)
                        + Math.pow((visionMeasuredPose.getY() - swerve.getPose().getY()), 2));
        if (distance <= maxVisionPoseOverrideDistance) {
            swerve.swerveDrivePoseEstimator.addVisionMeasurement(visionMeasuredPose, Timer.getFPGATimestamp());
        }
    }

    /**
     * Provide a heading for the robot to point at automatically.
     * Rotation directive is provided by this automatic pointing until the angle is reached or the rotation stick receives input.
     * @param angleDeg The heading to point at in degrees. 0/360 is the red alliance wall and increases CCW.
     */
    public void setTargetAngle(double angleDeg) {
        rotationController.reset();
        targetAngle = angleDeg;
    }

    /** Used for PathPlanner autonomous */
    public Pose2d getPose() {
        return swerve.getPose();
    }

    /** Used for PathPlanner autonomous */
    public void resetOdometry(Pose2d override) {
        swerve.resetOdometry(override);
    }

    /** Used for PathPlanner autonomous */
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

    public static class DriveSubsystemBuilder {
        private double maxTranslationVelocity;
        private double maxRotationVelocity;

        private double steeringGearRatio;
        private double steeringPulsePerRotation;

        private double driveGearRatio;
        private double drivePulsePerRotation;

        private double wheelDiameter;

        private double pointingP;
        private double pointingI;
        private double pointingD;
        private double pointingTolerance;

        private double maxVisionPoseOverrideDistance;

        private boolean maxVelocitiesSet = false;
        private boolean steeringPropertiesSet = false;
        private boolean drivePropertiesSet = false;
        private boolean wheelPropertiesSet = false;
        private boolean pointingPidSet = false;
        private boolean visionPropertiesSet = false;

        /**
         * Creates a builder to use
         */
        public DriveSubsystemBuilder() {}

        /**
         * Must be called to build a DriveSubsystem
         * @param maxTranslationVelocity Maximum translation velocity of the robot in meters per second
         * @param maxRotationVelocity Maximum rotation velocity of the robot in radians per second
         */
        public DriveSubsystemBuilder withMaxVelocities(double maxTranslationVelocity, double maxRotationVelocity) {
            this.maxTranslationVelocity = maxTranslationVelocity;
            this.maxRotationVelocity = maxRotationVelocity;
            maxVelocitiesSet = true;
            return this;
        }

        /**
         * Must be called to build a DriveSubsystem
         * @param steeringGearRatio Steering gear ratio (for reference, 12.8 in 2024)
         * @param steeringPulsePerRotation PPR of the steering motor (1 if integrated, 1 in 2024)
         * @return
         */
        public DriveSubsystemBuilder withSteeringMotorProperties(
                double steeringGearRatio, double steeringPulsePerRotation) {
            this.steeringGearRatio = steeringGearRatio;
            this.steeringPulsePerRotation = steeringPulsePerRotation;
            steeringPropertiesSet = true;
            return this;
        }

        /**
         * Must be called to build a DriveSubsystem
         * @param driveGearRatio Drive gear ratio (for reference, 6.12 in 2024)
         * @param drivePulsePerRotation PPR of the drive motor (1 if integrated, 1 in 2024)
         * @return
         */
        public DriveSubsystemBuilder withDriveMotorProperties(double driveGearRatio, double drivePulsePerRotation) {
            this.driveGearRatio = driveGearRatio;
            this.drivePulsePerRotation = drivePulsePerRotation;
            drivePropertiesSet = true;
            return this;
        }

        /**
         * Must be called to build a DriveSubsystem
         * @param wheelDiameter Wheel diameter in meters
         * @return
         */
        public DriveSubsystemBuilder withWheelDiameter(double wheelDiameter) {
            this.wheelDiameter = wheelDiameter;
            wheelPropertiesSet = true;
            return this;
        }

        /**
         * Must be called to build a DriveSubsystem
         * @param pointingP P for automatic pointing rotation
         * @param pointingI I for automatic pointing rotation
         * @param pointingD D for automatic pointing rotation
         * @param pointingTolerance Tolerance for automatic pointing, in degrees.
         * @return
         */
        public DriveSubsystemBuilder withPointingPID(
                double pointingP, double pointingI, double pointingD, double pointingTolerance) {
            this.pointingP = pointingP;
            this.pointingI = pointingI;
            this.pointingD = pointingD;
            this.pointingTolerance = pointingTolerance;
            pointingPidSet = true;
            return this;
        }

        /**
         * Must be called to build a DriveSubsystem
         * @param maxVisionPoseOverrideDistance Maximum vision pose difference allowed, in meters. (If the vision pose is too far off, ignore it)
         * @return
         */
        public DriveSubsystemBuilder withVisionProperties(double maxVisionPoseOverrideDistance) {
            this.maxVisionPoseOverrideDistance = maxVisionPoseOverrideDistance;
            visionPropertiesSet = true;
            return this;
        }

        /**
         * Builds a DriveSubsystem as long as all the builder methods have been called.
         * @return
         * @throws RuntimeException if all the builder methods have not been called
         */
        public DriveSubsystem build() {
            if (maxVelocitiesSet
                    && steeringPropertiesSet
                    && drivePropertiesSet
                    && wheelPropertiesSet
                    && pointingPidSet
                    && visionPropertiesSet) {
                return new DriveSubsystem(this);
            } else {
                // This is an unchecked exception that Java doesn't force us to catch
                // that's fine because it should never happen, if it does happen the robot should crash.
                throw new RuntimeException("Didn't fully initialize the builder");
            }
        }
    }
}
