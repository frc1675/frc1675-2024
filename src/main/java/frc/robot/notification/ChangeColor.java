package frc.robot.notification;

import edu.wpi.first.wpilibj2.command.Command;

public class ChangeColor extends Command {
  private LEDSubsystem subystem;
  
  public ChangeColor(){
    subystem = new LEDSubsystem();
  }

  @Override
  public void initialize(){
    subystem.handleStateUpdate(LEDStateEnum.COLOR_GREEN_SPARK);
  }

  @Override
  public void execute(){

  }

  @Override
  public void end(boolean interrupeted){

  }

}
