package frc.robot.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private double targetAngle = Constants.Arm.HOME_POSITION;
    private ProfiledPIDController pid;
    private boolean broken;
    private ShuffleboardTab dashboard;
    private IArmIO armIO;
    private TrapezoidProfile.Constraints profileConstraints;

    public Arm(IArmIO armIO) {
        this.armIO = armIO;
        // pid = new PIDController(Constants.Arm.PID_P_COEFFICIENT,
        // Constants.Arm.PID_I_COEFFICIENT, Constants.Arm.PID_D_COEFFICIENT);
        profileConstraints = new TrapezoidProfile.Constraints(Constants.Arm.MAXIMUM_VELOCITY,
                Constants.Arm.MAXIMUM_ACCELERATION);
        pid = new ProfiledPIDController(Constants.Arm.PID_P_COEFFICIENT, Constants.Arm.PID_I_COEFFICIENT,
                Constants.Arm.PID_D_COEFFICIENT, profileConstraints);
        pid.reset(armIO.getMeasurement());
        broken = false;
        initDashboard();
    }

    public double getAngle() {
        return armIO.getMeasurement();
    }

    public boolean isOnTarget() {
        return ((getAngle() >= targetAngle - Constants.Arm.TARGET_RANGE)
                && (getAngle() <= targetAngle + Constants.Arm.TARGET_RANGE));
    }

    public double getTarget() {
        return targetAngle;
    }

    public void setTarget(double inputTarget) {
        if (inputTarget <= Constants.Arm.MAX_ARM_RANGE_DEGREES) {
            targetAngle = Constants.Arm.MAX_ARM_RANGE_DEGREES;
        } else {
            targetAngle = inputTarget;
        }
    }

    public boolean isAtHomePostion() {
        return armIO.atFrontLimit();
    }

    public boolean isAtAmpPosition() {
        return isOnTarget() && armTargetName().equals("Amp");
    }

    public boolean isBroken() {
        return broken;
    }

    private String armTargetName() {
        if (targetAngle == Constants.Arm.HIGH_SCORE_POSITION) {
            return "High Goal";
        } else if (targetAngle == Constants.Arm.HOME_POSITION) {
            return "Home";
        } else if (targetAngle == Constants.Arm.AMP_POSITION) {
            return "Amp";
        } else {
            return "Undefined Target";
        }
    }

    private void initDashboard() {
        dashboard = Shuffleboard.getTab("Arm");
        dashboard.addDouble("Goal Angle", () -> getTarget());
        dashboard.addDouble("Angle Setpoint", () -> getPositionSetpoint());
        dashboard.addDouble("Velocity Setpoint", () -> getVelocitySetpoint());
        dashboard.addDouble("Arm Angle", () -> getAngle());
        dashboard.addBoolean("On Target", () -> isOnTarget());
        dashboard.addString("Target", () -> armTargetName());
        dashboard.addDouble("Motor Speed", () -> armIO.getMotorSpeed());
        dashboard.addBoolean("Home Switch", () -> isAtHomePostion());
        dashboard.addBoolean("Right home switch: ", () -> armIO.getRightHomeSwitch());
        dashboard.addBoolean("Left home switch: ", () -> armIO.getLeftHomeSwitch());
        dashboard.addBoolean("Is Broken", () -> isBroken());
        dashboard.add(pid);
    }

    public double getPositionSetpoint() {
        return pid.getSetpoint().position;
    }

    public double getVelocitySetpoint() {
        return -1.0 * pid.getSetpoint().velocity;
    }

    @Override
    // arm moves up as angle decreases and vice versa
    public void periodic() {

        armIO.periodic();
        double motorPower = -1.0 * pid.calculate(getAngle(), targetAngle);

        if (armIO.atFrontLimit()) {
            // Check if encoder and home switch disagree
            if (getAngle() < Constants.Arm.HOME_POSITION - Constants.Arm.TARGET_RANGE) {
                // Assume a sensor is broken, do not move arm.
                armIO.setMotorPower(0);
                broken = true;
            } else if (motorPower > 0) {
                // Only allow the motors to move away from the switch
                armIO.setMotorPower(motorPower);
            } else {
                // Stop arm if already moving in incorrect direction
                armIO.setMotorPower(0);
            }
        } else {
            if (getAngle() < Constants.Arm.MAX_ARM_RANGE_DEGREES) {
                // Arm is outside of desired range, only let it go back in.
                if (motorPower < 0) {
                    armIO.setMotorPower(motorPower);
                } else {
                    // Stop arm if already moving in incorrect direction
                    armIO.setMotorPower(0);
                }
            } else {
                // Arm within safe range, do what it wants (vast majority of time)
                armIO.setMotorPower(motorPower);
            }

        }
    }
}