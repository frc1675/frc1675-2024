package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Undertaker extends SubsystemBase{
    
    private CANSparkMax intakeMotor = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
    private CANSparkMax intakeMotor2 = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);
    
    public void undertakerStop(){
        intakeMotor.set(0);
        intakeMotor2.set(0);
    }
    
      public void noteRelease(double speed){
        intakeMotor.set(speed);
        intakeMotor2.set(speed);
    }
    
      public void notePickup(double speed){
        intakeMotor.set(speed * -1);
        intakeMotor2.set(speed * -1);
    }
}
