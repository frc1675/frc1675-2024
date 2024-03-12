package frc.robot.shooter.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class ShooterIntake extends Command {
  private final ShooterSubsystem subsystem;
  private final BooleanSupplier readyToIntake;

  public ShooterIntake(ShooterSubsystem subsystem, BooleanSupplier readyToIntake) {
    this.subsystem = subsystem;
    this.readyToIntake = readyToIntake;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setIndexerSpeed(Constants.Shooter.INTAKE_SPEED);
  }

  @Override
  public void execute() {
    if (readyToIntake.getAsBoolean() && !subsystem.isIndexerLoaded()) {
      subsystem.setIndexerSpeed(Constants.Shooter.INTAKE_SPEED);
    } else {
      subsystem.setIndexerSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
