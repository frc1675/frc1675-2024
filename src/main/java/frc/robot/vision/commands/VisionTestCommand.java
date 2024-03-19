package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.VisionSubsystem.LEDMode;

public class VisionTestCommand extends Command {

  private final VisionSubsystem vision;
  
  public VisionTestCommand(VisionSubsystem vision){
    addRequirements(vision);
    this.vision = vision;
  }

  @Override
  public void execute(){
    if(vision.hasTarget() && vision.getLEDMode() == LEDMode.OFF){
      vision.setLEDMode(LEDMode.BLINK);
    }else if(!vision.hasTarget() && vision.getLEDMode() != LEDMode.OFF){
      vision.setLEDMode(LEDMode.OFF);
    }
  }

}
