package frc.robot.shooter.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class SpinUp extends Command {
  private final ShooterSubsystem subsystem;
  private final BooleanSupplier slowShoot;
  private final double targetSpeed;

  public SpinUp(ShooterSubsystem subsystem, double targetSpeed, BooleanSupplier slowShoot) {
    this.subsystem = subsystem;
    this.slowShoot = slowShoot;
    this.targetSpeed = targetSpeed;
    addRequirements(subsystem);
  }

  public SpinUp(ShooterSubsystem subsystem, double targetSpeed) {
    this.subsystem = subsystem;
    this.slowShoot = () -> false;
    this.targetSpeed = targetSpeed;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double scale = slowShoot.getAsBoolean() ? Constants.Shooter.AMP_SHOOT_SCALE : 1;
    subsystem.setTargetShooterSpeed(targetSpeed * scale);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return subsystem.isShooterReady();
  }
}
