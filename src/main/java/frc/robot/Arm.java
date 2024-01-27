package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;  

public class Arm extends SubsystemBase{
    private double targetAngle;
    private Encoder armEncoder;

    private CANSparkMax motorOne;
    private CANSparkMax motorTwo;
    private PIDController pid;
    private DigitalInput homeSwitch;

    public Arm(){
        homeSwitch = new DigitalInput(Constants.Arm.DIGITAL_INPUT_CHANNEL);
        pid = new PIDController(Constants.Arm.P_COEFFICIENT, Constants.Arm.I_COEFFICIENT, Constants.Arm.D_COEFFICIENT);
        armEncoder = new Encoder(Constants.Arm.ENCODER_CHANNEL_A_ID, Constants.Arm.ENCODER_CHANNEL_B_ID);
        motorOne = new CANSparkMax(Constants.Arm.ARM_MOTOR_ONE, MotorType.kBrushless);
        motorTwo = new CANSparkMax(Constants.Arm.ARM_MOTOR_TWO, MotorType.kBrushless);
        if (Robot.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(motorOne, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorTwo, DCMotor.getNEO(1));
        }
    }
    public double getAngle(){
        return ((double) armEncoder.get()/Constants.Arm.ENCODER_COUNT)*360;
    }

    public boolean isOnTarget(){
        return((targetAngle - Constants.Arm.TARGET_RANGE_DEGREES <= getAngle()) && (targetAngle + Constants.Arm.TARGET_RANGE_DEGREES >= getAngle()));
    }

    public double getTarget(){
        return targetAngle;
    }

    public void setTarget(double inputTarget){
        targetAngle = inputTarget;
    }
    public boolean isAtHomePostion() {
        if (homeSwitch.get()){
            return true;
        } else{
            return((Constants.Arm.HOME_POSITION - Constants.Arm.HOME_POSITION_RANGE_DEGREES <= getAngle()) && (Constants.Arm.HOME_POSITION + Constants.Arm.HOME_POSITION_RANGE_DEGREES >= getAngle()));

        }
    }

    @Override
    public void periodic(){
        double motorVoltage = pid.calculate(targetAngle - getAngle());
        if (homeSwitch.get()){
            if (motorVoltage >= 0){
                motorOne.setVoltage(motorVoltage);
                motorTwo.setVoltage(motorVoltage);
            }
        } else{
            motorOne.setVoltage(motorVoltage);
            motorTwo.setVoltage(motorVoltage);
        }
    }
}