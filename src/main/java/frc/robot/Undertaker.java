package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Undertaker extends SubsystemBase{
    
    private CANSparkMax intakeMotorOne = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
    private CANSparkMax intakeMotorTwo = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);
    
    public class Undertakers {
      public static final int INTAKE_MOTOR_ONE = 9;
      public static final int INTAKE_MOTOR_TWO = 10;
    }

    public void undertakerControls(double speed){
        //stop
        intakeMotorOne.set(0);
        intakeMotorTwo.set(0);

        //forwards
        intakeMotorOne.set(speed);
        intakeMotorTwo.set(speed);

        //backwards
        intakeMotorOne.set(speed * -1);
        intakeMotorTwo.set(speed * -1);
    }
    
/** 
      public void noteRelease(double speed){
        intakeMotorOne.set(speed);
        intakeMotorTwo.set(speed);
    }
    
      public void notePickup(double speed){
        intakeMotorOne.set(speed * -1);
        intakeMotorTwo.set(speed * -1);
    }
    */
}
