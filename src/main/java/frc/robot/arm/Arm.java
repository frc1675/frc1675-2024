package frc.robot.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private double targetAngle;
    private ProfiledPIDController pid;
    private boolean broken;
    private ShuffleboardTab dashboard;
    private IArmIO armIO;
    private TrapezoidProfile.Constraints profileConstraints;

    public Arm(IArmIO armIO) {
        this.armIO = armIO;
        //pid = new PIDController(Constants.Arm.PID_P_COEFFICIENT, Constants.Arm.PID_I_COEFFICIENT, Constants.Arm.PID_D_COEFFICIENT);
        profileConstraints = new TrapezoidProfile.Constraints(130, 650);
        pid = new ProfiledPIDController(Constants.Arm.PID_P_COEFFICIENT, Constants.Arm.PID_I_COEFFICIENT, Constants.Arm.PID_D_COEFFICIENT, profileConstraints);
        broken = false;
        initDashboard();
    }

    public double getAngle() {
        return armIO.getMeasurement();
    }

    public boolean isOnTarget() {
        return ((getAngle() >= targetAngle - Constants.Arm.TARGET_RANGE) && (getAngle() <= targetAngle+Constants.Arm.TARGET_RANGE));
    }

    public double getTarget() {
        return targetAngle;
    }

    public void setTarget(double inputTarget) {
        if (inputTarget >= Constants.Arm.MAX_ARM_RANGE_DEGREES) {
            targetAngle = Constants.Arm.MAX_ARM_RANGE_DEGREES;
        } else {
            targetAngle = inputTarget;
        }
    }

    public boolean isAtHomePostion() {
        return armIO.atFrontLimit();
    }

    public boolean isBroken() {
        return broken;
    }

    private String armTargetName() {
        if (targetAngle == Constants.Arm.HIGH_SCORE_POSITION){
            return "High Goal";
        } else if (targetAngle == Constants.Arm.LOW_SCORE_POSITION){
            return "Low Goal";
        } else if (targetAngle == Constants.Arm.HOME_POSITION) {
            return "Home";
        } else{
            return "Undefined Target";
        }
    }

    private void initDashboard() {
        dashboard = Shuffleboard.getTab("Arm");
        dashboard.addDouble("Target Angle", () -> getTarget());
        dashboard.addDouble("Arm Angle", () -> getAngle());
        dashboard.addBoolean("On Target", () -> isOnTarget());
        dashboard.addString("Target", () -> armTargetName());
        dashboard.addDouble("Motor Speed", () -> armIO.getMotorSpeed());
        dashboard.add(pid).withWidget(BuiltInWidgets.kPIDController); 
    }

    @Override
    public void periodic() {
        armIO.periodic();
        double motorPower = pid.calculate(getAngle(), targetAngle);

        if (armIO.atFrontLimit()) {
            // Check if encoder and home switch disagree
            if (getAngle() > Constants.Arm.HOME_POSITION + Constants.Arm.TARGET_RANGE) {
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
            if (getAngle() > Constants.Arm.MAX_ARM_RANGE_DEGREES) {
                // Arm is outside of desired range, only let it go back in.
                if (motorPower < 0) {
                    armIO.setMotorPower(motorPower);
                } else {
                    // Stop arm if already moving in incorrect direction
                    armIO.setMotorPower(0);
                }
            } else {
                // Arm within safe range, do what it wants
                armIO.setMotorPower(motorPower);
            }

        }
    }
}