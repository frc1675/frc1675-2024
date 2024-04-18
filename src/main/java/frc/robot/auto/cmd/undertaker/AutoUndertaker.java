package frc.robot.auto.cmd.undertaker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.undertaker.UndertakerSubsystem;
import java.util.function.BooleanSupplier;

/**
 * Run the undertaker at the given speed, unless the boolean supplier is true. This command never
 * ends.
 */
public class AutoUndertaker extends Command {

  private final UndertakerSubsystem undertaker;
  private final double speed;
  private final BooleanSupplier shouldStop;

  public AutoUndertaker(UndertakerSubsystem undertaker, double speed, BooleanSupplier shouldStop) {
    this.undertaker = undertaker;
    this.speed = speed;
    this.shouldStop = shouldStop;
    addRequirements(undertaker);
  }

  @Override
  public void execute() {
    if (shouldStop.getAsBoolean()) {
      undertaker.run(0);
    } else {
      undertaker.run(speed);
    }
  }

  public void end(boolean interrupted) {
    undertaker.run(0);
  }

  @Override
  public boolean isFinished() {
    return shouldStop.getAsBoolean();
  }
}
