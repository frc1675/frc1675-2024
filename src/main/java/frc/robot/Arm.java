package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private double targetAngle;
    private Encoder armEncoder;

    private CANSparkMax motorOne;
    private CANSparkMax motorTwo;
    private PIDController pid;
    private DigitalInput homeSwitch;
    private boolean broken;
    private EncoderSim simEncoder;
    private ShuffleboardTab dashboard;

    public Arm() {
        homeSwitch = new DigitalInput(Constants.Arm.HOMESWITCH_DIGITAL_INPUT_CHANNEL);
        pid = new PIDController(Constants.Arm.PID_CONTROLLER_P_COEFFICIENT, Constants.Arm.PID_CONTROLLER_I_COEFFICIENT, Constants.Arm.PID_CONTROLLER_D_COEFFICIENT);
        armEncoder = new Encoder(Constants.Arm.ENCODER_CHANNEL_A, Constants.Arm.ENCODER_CHANNEL_B);
        motorOne = new CANSparkMax(Constants.Arm.ARM_MOTOR_ONE, MotorType.kBrushless);
        motorTwo = new CANSparkMax(Constants.Arm.ARM_MOTOR_TWO, MotorType.kBrushless);
        broken = false;
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motorOne, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorTwo, DCMotor.getNEO(1));
            simEncoder = new EncoderSim(armEncoder);
        }
        initDashboard();
    }

    public double getAngle() {
        return ((double) simEncoder.getCount() / Constants.Arm.ENCODER_COUNT) * 360;
    }

    public boolean isOnTarget() {
        return ((targetAngle - Constants.Arm.TARGET_MOVEMENT_RANGE_DEGREES <= getAngle()) && (targetAngle + Constants.Arm.TARGET_MOVEMENT_RANGE_DEGREES >= getAngle()));
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
        return homeSwitch.get();
    }

    public boolean isBroken() {
        return broken;
    }

    private String armPositionName() {
        if (targetAngle == Constants.Arm.HIGH_SCORE_POSITION){
            return ("High Goal");
        } else if (targetAngle == Constants.Arm.LOW_SCORE_POSITIOIN){
            return ("Low Goal");
        } else if (targetAngle == Constants.Arm.HOME_POSITION) {
            return ("Home");
        } else{
            return ("Moving...");
        }
    }

    private boolean armPositionStatus() {
        if (targetAngle == Constants.Arm.HIGH_SCORE_POSITION || targetAngle == Constants.Arm.LOW_SCORE_POSITIOIN || targetAngle == Constants.Arm.HOME_POSITION){
            return true;
        } else{
            return false;
        }
    }

    private void initDashboard() {
        dashboard = Shuffleboard.getTab("Arm");
        dashboard.addDouble("Target Angle", () -> getTarget());
        dashboard.addDouble("Arm Angle", () -> getAngle());
        dashboard.addBoolean("On Target", () -> armPositionStatus());
        dashboard.addString("Position", () -> armPositionName());

    }

    @Override
    public void periodic() {
        double motorVoltage = pid.calculate(targetAngle - getAngle());

        if (homeSwitch.get()) {
            // Check if encoder and home switch disagree
            if (getAngle() > Constants.Arm.HOME_SWITCH_FAILSAFE_DEGREES) {
                // Assume a sensor is broken, do not move arm.
                motorOne.setVoltage(0);
                motorTwo.setVoltage(0);
                broken = true;
            } else if (motorVoltage > 0) {
                // Only allow the motors to move away from the switch
                motorOne.setVoltage(motorVoltage);
                motorTwo.setVoltage(motorVoltage);
            } else {
                // Stop arm if already moving in incorrect direction
                motorOne.setVoltage(0);
                motorTwo.setVoltage(0);
            }
        } else {
            if (getAngle() > Constants.Arm.ARM_ANGLE_LIMIT_DEGREES) {
                // Arm is outside of desired range, only let it go back in.
                if (motorVoltage < 0) {
                    motorOne.setVoltage(motorVoltage);
                    motorTwo.setVoltage(motorVoltage);
                } else {
                    // Stop arm if already moving in incorrect direction
                    motorOne.setVoltage(0);
                    motorTwo.setVoltage(0);
                }
            } else {
                // Arm within safe range, do what it wants
                motorOne.setVoltage(motorVoltage);
                motorTwo.setVoltage(motorVoltage);
            }

        }
    }
}