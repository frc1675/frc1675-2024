package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class RealArmIO implements IArmIO {

    private CANSparkMax motorOne;
    private CANSparkMax motorTwo;
    private Encoder encoder;
    private DigitalInput homeSwitch;

    public RealArmIO() {
        motorOne = new CANSparkMax(Constants.Arm.ARM_MOTOR_ONE, MotorType.kBrushless);
        motorTwo = new CANSparkMax(Constants.Arm.ARM_MOTOR_TWO, MotorType.kBrushless);
        encoder = new Encoder(Constants.Arm.ENCODER_CHANNEL_A, Constants.Arm.ENCODER_CHANNEL_B);
        homeSwitch = new DigitalInput(Constants.Arm.HOMESWITCH_CHANNEL);
        motorOne.setInverted(true);
    }

    @Override
    public void setMotorPower(double power){
        motorOne.setVoltage(power*12);
        motorTwo.setVoltage(power*12);
    }

    @Override
    public double getMeasurement(){
        return ((double) encoder.get() / Constants.Arm.ENCODER_COUNT) * 360;
    }

    @Override
    public double getMotorSpeed(){
        return motorOne.get();
    }

    @Override
    public boolean atFrontLimit(){
        return homeSwitch.get();
    }

    @Override
    public void periodic(){

    }
}