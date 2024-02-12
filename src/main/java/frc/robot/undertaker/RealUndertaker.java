package frc.robot.undertaker;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.FaultID;
import frc.robot.Constants;


public class RealUndertaker implements IUndertaker {

  private CANSparkMax intakeMotorOne = new CANSparkMax(Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
  private CANSparkMax intakeMotorTwo = new CANSparkMax(Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);
  private double desiredSpeed;

  public RealUndertaker(){}

  @Override
  public double[] getMotorsOutput(){
    double[] motorsOutput = new double[2];
    motorsOutput[0] = intakeMotorOne.getAppliedOutput();
    motorsOutput[1] = intakeMotorTwo.getAppliedOutput();
    return motorsOutput;
  }

  @Override
  public void run(double speed){
    desiredSpeed = speed;
    intakeMotorOne.setVoltage(speed * 12.0);
    intakeMotorTwo.setVoltage(speed * 12.0);
  }

  @Override
  public double getDesiredSpeed(){
    return desiredSpeed;
  }

  @Override
  public boolean[] isAlive() {
    boolean[] motorStatus = new boolean[2];
    if(!intakeMotorOne.getFault(FaultID.kCANRX) && !intakeMotorOne.getFault(FaultID.kCANTX)){
      motorStatus[0] = true;
    }
    if(!intakeMotorTwo.getFault(FaultID.kCANRX) && !intakeMotorTwo.getFault(FaultID.kCANTX)){
      motorStatus[1] = true;
    }
    if(intakeMotorOne.getFault(FaultID.kCANRX) && intakeMotorOne.getFault(FaultID.kCANTX)){
      motorStatus[0] = false;  
    }
    if(intakeMotorTwo.getFault(FaultID.kCANRX) && intakeMotorTwo.getFault(FaultID.kCANTX)){
      motorStatus[1] = false; 
    }
    return motorStatus;
  }

}
