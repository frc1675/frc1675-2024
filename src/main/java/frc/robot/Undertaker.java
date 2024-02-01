package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Undertaker extends SubsystemBase{
    
    private CANSparkMax intakeMotorOne = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
    private CANSparkMax intakeMotorTwo = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);

    public Undertaker(){
        if (Robot.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(intakeMotorOne, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(intakeMotorTwo, DCMotor.getNEO(1));
        }
    }

    public void undertakerControl(double speed){
        intakeMotorOne.setVoltage(speed);
        intakeMotorTwo.setVoltage(speed);

    }

}
 