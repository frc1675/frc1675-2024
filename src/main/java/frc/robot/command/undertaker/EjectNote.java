package frc.robot.command.undertaker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Undertaker;

public class EjectNote extends Command {
    
     private Undertaker undertaker;



  public EjectNote(Undertaker undertaker) {
    addRequirements(undertaker);
    this.undertaker = undertaker;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    undertaker.run(Constants.Undertaker.EJECT_VOLTAGE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    undertaker.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
