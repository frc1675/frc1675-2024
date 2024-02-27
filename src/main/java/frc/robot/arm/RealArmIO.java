package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class RealArmIO implements IArmIO {

    private CANSparkMax armMotorRight;
    private CANSparkMax armMotorLeft;
    private Encoder encoder;
    private DigitalInput homeSwitch;

    public RealArmIO() {
        armMotorRight = new CANSparkMax(Constants.Arm.ARM_MOTOR_RIGHT, MotorType.kBrushless);
        armMotorLeft = new CANSparkMax(Constants.Arm.ARM_MOTOR_LEFT, MotorType.kBrushless);
        armMotorRight.setInverted(true);
        armMotorLeft.setInverted(false);
        encoder = new Encoder(Constants.Arm.ENCODER_CHANNEL_A, Constants.Arm.ENCODER_CHANNEL_B);
        homeSwitch = new DigitalInput(Constants.Arm.HOMESWITCH_CHANNEL);    }

    @Override
    public void setMotorPower(double power){
        armMotorRight.setVoltage(power*12);
        armMotorLeft.setVoltage(power*12);
    }

    @Override
    public double getMeasurement(){
        return ((double) encoder.get() / Constants.Arm.ENCODER_COUNT) * 360;
    }

    @Override
    public double getMotorSpeed(){
        return armMotorRight.get();
    }

    @Override
    public boolean atFrontLimit(){
        return !(homeSwitch.get());
    }

    @Override
    public void periodic(){

    }
}