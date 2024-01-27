package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Arm;
public class MoveToPosition extends Command {
  private Arm arm;
  private double targetAngle;
  public MoveToPosition(Arm arm, double targetAngle){
    this.targetAngle = targetAngle;
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setTarget(targetAngle);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.isOnTarget();
  }
}
