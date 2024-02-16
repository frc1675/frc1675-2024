package frc.robot.undertaker;

import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class SimUndertaker implements IUndertaker {

  private final CANSparkMax intakeMotorOne = new CANSparkMax(Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
  private final CANSparkMax intakeMotorTwo = new CANSparkMax(Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);
  private boolean[] mockMotorStatus = { false, false }; 
  private double desiredSpeed;
  private boolean hasRun = false;

  public SimUndertaker(){
    REVPhysicsSim.getInstance().addSparkMax(intakeMotorOne, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(intakeMotorTwo, DCMotor.getNEO(1));
  }

  @Override
  public double getDesiredSpeed(){
    return desiredSpeed;
  }

  @Override
  public double[] getMotorsOutput(){
    double[] motorsOutput = new double[2];
    motorsOutput[0] = intakeMotorOne.getAppliedOutput();
    motorsOutput[1] = intakeMotorTwo.getAppliedOutput();
    return motorsOutput;
  }

  @Override
  public void run(double speed){
    if (mockMotorStatus[0] && mockMotorStatus[1]) {
      this.desiredSpeed = speed;
      intakeMotorOne.setVoltage(speed * 12.0);
      intakeMotorTwo.setVoltage(speed * 12.0);
      hasRun = true;
    }
  }

  @Override
  public boolean[] isAlive(){
    return mockMotorStatus;
  }

  public void setIsAlive(boolean[] status){
    mockMotorStatus = status;
  }

  public boolean getHasRun(){
    return hasRun;
  }

  public void resetHasRun(){
    hasRun = false;
  }
}
