package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
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
        homeSwitch = new DigitalInput(0);
        pid = new PIDController(Constants.Arm.P_COEFFICIENT, Constants.Arm.I_COEFFICIENT, Constants.Arm.D_COEFFICIENT);
        armEncoder = new Encoder(0,0);
        motorOne = new CANSparkMax(Constants.Arm.MOTOR_1_ID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        motorTwo = new CANSparkMax(Constants.Arm.MOTOR_2_ID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    }

    public double getAngle(){
        return (armEncoder.get()/Constants.Arm.ENCODER_COUNT)*360;
    }

    public boolean isOnTarget(){
        return((targetAngle - Constants.Arm.TARGET_RANGE <= getAngle()) && (targetAngle + Constants.Arm.TARGET_RANGE >= getAngle()));
    }

    public double getTarget(){
        return targetAngle;
    }

    public void setTarget(double inputTarget){
        targetAngle = inputTarget;
    }
    public boolean isAtHomePostion() {
        return((Constants.Arm.HOME_POSITION - Constants.Arm.HOME_POSITION_RANGE <= getAngle()) && (Constants.Arm.HOME_POSITION + Constants.Arm.HOME_POSITION_RANGE >= getAngle()));
    }

    @Override
    public void periodic(){
        double motorSpeed = pid.calculate(targetAngle - getAngle());
        if (homeSwitch.get()){
            if (motorSpeed >= 0){
                motorOne.set(motorSpeed);
                motorTwo.set(motorSpeed);
            }
        } else{
            motorOne.set(motorSpeed);
            motorTwo.set(motorSpeed);
        }
    }
}