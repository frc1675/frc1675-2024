package frc.robot.cmdGroup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.MoveToPosition;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.commands.Shoot;
import frc.robot.shooter.commands.SpinUp;
import frc.robot.vision.VisionSubsystem;

public class LongShot extends SequentialCommandGroup {
  public LongShot(ShooterSubsystem shooter, ArmSubsystem arm, VisionSubsystem vision) {
    Rotation2d measurement = vision.getVerticalSpeakerOffset();
    if (measurement == null) {
      return;
    }
    double aimAngle = ArmSubsystem.calcSpeakerArmAngle(VisionSubsystem.getDistanceToSpeaker(measurement.getDegrees()));
    
    addCommands(
        new MoveToPosition(arm, Constants.Arm.HOME_POSITION - aimAngle),
        new SpinUp(shooter, Constants.Shooter.LONG_SHOT_SPEED, Constants.Shooter.LONG_SHOT_SPEED),
        new Shoot(shooter).withTimeout(Constants.Shooter.SHOOTER_SHOOT_TIME),
        new MoveToPosition(arm, Constants.Arm.HOME_POSITION));
  }
}
