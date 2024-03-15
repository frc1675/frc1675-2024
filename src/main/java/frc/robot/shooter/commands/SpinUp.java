package frc.robot.shooter.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class SpinUp extends Command {
  private final ShooterSubsystem subsystem;
  private final BooleanSupplier slowShoot;
  private final double targetTopSpeed;
  private final double targetBottomSpeed;

  public SpinUp(ShooterSubsystem subsystem, double targetTopSpeed, double targetBottomSpeed, BooleanSupplier slowShoot) {
    this.subsystem = subsystem;
    this.slowShoot = slowShoot;
    this.targetTopSpeed = targetTopSpeed;
    this.targetBottomSpeed = targetBottomSpeed;
    addRequirements(subsystem);
  }

  public SpinUp(ShooterSubsystem subsystem, double targetTopSpeed, double targetBottomSpeed) {
    this.subsystem = subsystem;
    this.slowShoot = () -> false;
    this.targetTopSpeed = targetTopSpeed;
    this.targetBottomSpeed = targetBottomSpeed;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double scale = slowShoot.getAsBoolean() ? Constants.Shooter.AMP_SHOOT_SCALE : 1;
    subsystem.setTargetShooterSpeeds(targetTopSpeed * scale, targetBottomSpeed * scale);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return subsystem.isShooterReady();
  }
}
