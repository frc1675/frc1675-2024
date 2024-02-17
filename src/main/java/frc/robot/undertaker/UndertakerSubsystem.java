package frc.robot.undertaker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class UndertakerSubsystem extends SubsystemBase { 
    private final IUndertaker undertakerLogic;
    private final ShuffleboardTab undertakerTab; 

    public UndertakerSubsystem(IUndertaker undertakerLogic){
      this.undertakerLogic = undertakerLogic;    
      undertakerTab = Shuffleboard.getTab("Undertaker");
      undertakerTab.addDouble("Desired Speed: ", () -> undertakerLogic.getDesiredSpeed());
      undertakerTab.addDouble("Output Volts 1: ",() -> undertakerLogic.getMotorsOutput()[0]);
      undertakerTab.addDouble("Output Volts 2: ",() -> undertakerLogic.getMotorsOutput()[1]); 
      undertakerTab.addBoolean("Motor 1 Status", () -> undertakerLogic.isAlive()[0]);
      undertakerTab.addBoolean("Motor 2 Status", () -> undertakerLogic.isAlive()[1]);
    }

    public void run(double speed){
      undertakerLogic.run(speed);
    }

} 
