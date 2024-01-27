package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Undertaker extends SubsystemBase{
    
    private CANSparkMax intakeMotorOne = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
    private CANSparkMax intakeMotorTwo = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);

    public class Undertakers {}

    public void undertakerControl(double speed){
        intakeMotorOne.set(speed);
        intakeMotorTwo.set(speed);
    }
    
}
