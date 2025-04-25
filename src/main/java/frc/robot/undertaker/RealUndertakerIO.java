package frc.robot.undertaker;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class RealUndertakerIO implements IUndertaker {

    private boolean[] motorStatus = {true, true};
    private SparkMax intakeMotorOne = new SparkMax(Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
    private SparkMax intakeMotorTwo = new SparkMax(Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);
    private double desiredSpeed;

    public RealUndertakerIO() {
        intakeMotorOne.setInverted(true);
        intakeMotorTwo.setInverted(true);
    }

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
        // motorStatus[0] = !intakeMotorOne.getFault(FaultID.kCANRX) && !intakeMotorOne.getFault(FaultID.kCANTX);
        // motorStatus[1] = !intakeMotorTwo.getFault(FaultID.kCANRX) && !intakeMotorTwo.getFault(FaultID.kCANTX);
        motorStatus[0] = true;
        motorStatus[1] = true;
        return motorStatus;
    }
}
