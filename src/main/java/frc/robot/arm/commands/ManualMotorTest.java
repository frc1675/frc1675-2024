package frc.robot.arm.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.arm.Arm;
public class ManualMotorTest extends Command {
  private Arm arm;
  private double power;
  public ManualMotorTest(Arm arm, double power){
    this.arm = arm;
    this.power = power;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setMotorManual(power);
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

