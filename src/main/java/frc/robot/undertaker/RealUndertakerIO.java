package frc.robot.undertaker;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class RealUndertakerIO implements IUndertaker {

    private boolean[] motorStatus = {true, true};
    private CANSparkMax intakeMotorOne = new CANSparkMax(Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
    private CANSparkMax intakeMotorTwo = new CANSparkMax(Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);
    private double desiredSpeed;

    public RealUndertakerIO() {}

    @Override
    public double[] getMotorsOutput() {
        double[] motorsOutput = new double[2];
        motorsOutput[0] = intakeMotorOne.getAppliedOutput();
        motorsOutput[1] = intakeMotorTwo.getAppliedOutput();
        return motorsOutput;
    }

    @Override
    public void run(double speed) {
        if (motorStatus[0] && motorStatus[1]) {
            desiredSpeed = speed;
            intakeMotorOne.setVoltage(speed * 12.0);
            intakeMotorTwo.setVoltage(speed * 8.0);
        }
    }

    @Override
    public double getDesiredSpeed() {
        return desiredSpeed;
    }

    @Override
    public boolean[] isAlive() {
        motorStatus[0] = !intakeMotorOne.getFault(FaultID.kCANRX) && !intakeMotorOne.getFault(FaultID.kCANTX);
        motorStatus[1] = !intakeMotorTwo.getFault(FaultID.kCANRX) && !intakeMotorTwo.getFault(FaultID.kCANTX);
        return motorStatus;
    }
}
