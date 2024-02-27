package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class RealArmIO implements IArmIO {

    private CANSparkMax armMotorRight;
    private CANSparkMax armMotorLeft;
    private DutyCycleEncoder encoder;
    private DigitalInput homeSwitch;
    private double motorPower;

    public RealArmIO() {
        armMotorRight = new CANSparkMax(Constants.Arm.ARM_MOTOR_RIGHT, MotorType.kBrushless);
        armMotorLeft = new CANSparkMax(Constants.Arm.ARM_MOTOR_LEFT, MotorType.kBrushless);
        armMotorRight.setInverted(true);
        armMotorLeft.setInverted(false);
        encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_CHANNEL);
        homeSwitch = new DigitalInput(Constants.Arm.HOMESWITCH_CHANNEL);    }

    @Override
    public void setMotorPower(double power){
        //postive power makes arm move away from home
        motorPower = power;
        //armMotorRight.setVoltage(power*12);
        armMotorLeft.setVoltage(power*12);
    }

    @Override
    public double getMeasurement(){
        return encoder.get() *360;
    }

    @Override
    public double getMotorSpeed(){
        return motorPower;
    }

    @Override
    public boolean atFrontLimit(){
        return !(homeSwitch.get());
    }

    @Override
    public void periodic(){

    }
}