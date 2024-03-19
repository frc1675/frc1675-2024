package frc.robot.arm.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.vision.VisionSubsystem;

public class SpeakerAlign extends Command {
  private ArmSubsystem arm;
  private VisionSubsystem vision;
  
  public SpeakerAlign(ArmSubsystem arm, VisionSubsystem vision) {
    this.arm = arm;
    this.vision = vision;
    addRequirements(arm, vision);
  }

  @Override
  public void initialize() {
    new MoveToPosition(arm, Constants.Arm.HOME_POSITION - vision.getVerticalSpeakerOffset().getDegrees());
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

