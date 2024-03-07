package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
<<<<<<< HEAD
import frc.robot.arm.ArmSubsystem;

public class MoveToHome extends Command {
  private ArmSubsystem arm;
  public MoveToHome(ArmSubsystem arm) {
=======
import frc.robot.arm.Arm;

public class MoveToHome extends Command {
  private Arm arm;
  public MoveToHome(Arm arm) {
>>>>>>> main
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setTarget(Constants.Arm.HOME_POSITION);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.isAtHomePostion();
  }
}
