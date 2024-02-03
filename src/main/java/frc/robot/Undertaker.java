package frc.robot;


import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Undertaker extends SubsystemBase{
    
    private CANSparkMax intakeMotorOne = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_ONE, MotorType.kBrushless);
    private CANSparkMax intakeMotorTwo = new CANSparkMax( Constants.Undertaker.INTAKE_MOTOR_TWO, MotorType.kBrushless);
    private ShuffleboardTab undertakerTab;
    private double speed;
   
    public Undertaker(){
        undertakerTab();

        if (Robot.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(intakeMotorOne, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(intakeMotorTwo, DCMotor.getNEO(1));
            
        }
    }

    public void run(double speed){
        this.speed = speed;
        intakeMotorOne.setVoltage(speed * 12.0);
        intakeMotorTwo.setVoltage(speed * 12.0);
    }

    private void undertakerTab(){
        undertakerTab = Shuffleboard.getTab("Undertaker");
        undertakerTab.addDouble("Desired Speed", () ->speed);
        undertakerTab.addDouble("Output Volts 1",() -> intakeMotorOne.getAppliedOutput());
        undertakerTab.addDouble("Output Volts 2",() ->  intakeMotorTwo.getAppliedOutput());
    }
  
    
    
    

}
 