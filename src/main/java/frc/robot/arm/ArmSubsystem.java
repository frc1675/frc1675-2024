package frc.robot.arm;

import java.util.HashMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
    private double targetAngle = Constants.Arm.HOME_POSITION;
    private ProfiledPIDController pid;
    private boolean broken;
    private ShuffleboardTab dashboard;
    private IArmIO armIO;
    private TrapezoidProfile.Constraints profileConstraints;

    private static HashMap<Integer, Double[]> speakerDistToAngleTable = new HashMap<Integer, Double[]>() {{
        // format: distance, {bottom shot, top shot}
        // uses arm angle values so high shot has a lower angle than low shot
        put(50, new Double[]{70.76, 58.19});
        put(90, new Double[]{87.85, 80.26});
        put(120, new Double[]{94.76, 89.50});
        put(150, new Double[]{99.20, 95.34});
        put(180, new Double[]{102.23, 99.26});
        put(210, new Double[]{104.42, 102.05});
        put(240, new Double[]{106.07, 104.11});
        put(270, new Double[]{107.36, 105.70});
        put(300, new Double[]{108.39, 106.96});
        put(318, new Double[]{108.91, 107.59});
    }};

    public static double calcSpeakerArmAngle(double hDist) {
        // find nearest pre-calculated values
        Integer lowMatch = -1000;
        Integer highMatch = 1000;

        for (Integer dist : speakerDistToAngleTable.keySet()) {
            if (dist >= lowMatch && dist <= hDist) {
                lowMatch = dist;
            } else if (dist <= highMatch && dist >= hDist) {
                highMatch = dist;
            }
        }

        double bottomShot;
        double topShot;

        // interpolate to estimate top and bottom angles
        double interpolant = MathUtil.inverseInterpolate(lowMatch, highMatch, hDist);
        bottomShot = MathUtil.interpolate(
            speakerDistToAngleTable.get(lowMatch)[0],
            speakerDistToAngleTable.get(highMatch)[0],
            interpolant);
        topShot = MathUtil.interpolate(
            speakerDistToAngleTable.get(lowMatch)[1],
            speakerDistToAngleTable.get(highMatch)[1],
            interpolant);
            
        // calculate desired angle based on range
        double dif = topShot - bottomShot;
        return bottomShot + dif * Constants.Arm.SPEAKER_SHOT_ANGLE_DIF_MULTIPLIER;
    }

    public static ArmSubsystem create() {
        return new ArmSubsystem(Robot.isReal() ? new RealArmIO() : new SimArmIO());
    }

    public ArmSubsystem(IArmIO armIO) {
        this.armIO = armIO;
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
        pid.setTolerance(Constants.Arm.TARGET_RANGE);
        return pid.atGoal();
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
        return isOnTarget() && getTarget() == Constants.Arm.AMP_POSITION;
    }

    public boolean isAtPodiumPosition() {
        return isOnTarget() && getTarget() == Constants.Arm.LONG_SHOT_ANGLE;
    }

    public boolean isBroken() {
        return broken;
    }


    private void initDashboard() {
        dashboard = Shuffleboard.getTab("Arm");
        dashboard.addDouble("Goal Angle", () -> getTarget());
        dashboard.addDouble("Angle Setpoint", () -> getPositionSetpoint());
        dashboard.addDouble("Velocity Setpoint", () -> getVelocitySetpoint());
        dashboard.addDouble("Arm Angle", () -> getAngle());
        dashboard.addBoolean("On Target", () -> isOnTarget());
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
        //Multiply by -1 to invert motorPower because positive motor power moves the arm upwards but decreases the angle read by the encoder and vice versa for negitive motor power. 
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