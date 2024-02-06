package frc.robot.notification;

import edu.wpi.first.wpilibj2.command.Command;

public class ChangeColor extends Command {
  private LEDSubsystem subystem;
  
  public ChangeColor(LEDSubsystem subsystem){
    this.subystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize(){
    subystem.changeColor(LEDStateEnum.GREEN);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
