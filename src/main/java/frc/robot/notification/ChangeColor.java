package frc.robot.notification;

import edu.wpi.first.wpilibj2.command.Command;

public class ChangeColor extends Command {
  private LEDSubsystem subystem;
  private LEDStateEnum color;
  
  public ChangeColor(LEDSubsystem subsystem, LEDStateEnum color){
    this.subystem = subsystem;
    this.color = color;
    addRequirements(subsystem);
  }

  @Override
  public void initialize(){
    subystem.changeColor(color);
  }

  @Override
  public void end(boolean inturr){
    subystem.changeColor(LEDStateEnum.RED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
