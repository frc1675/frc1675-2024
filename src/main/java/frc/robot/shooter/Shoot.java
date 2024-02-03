// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Shoot extends Command {
    private ShooterSubsystem subsystem;
    private double targetSpeed;
    private Timer timer;

    public Shoot(ShooterSubsystem subsystem, double targetSpeed) {
        this.subsystem = subsystem;
        this.targetSpeed = targetSpeed;
        timer = new Timer();
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.setIndexerSpeed(targetSpeed);
        timer.start();  
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.setTargetShooterSpeed(0);
        subsystem.setIndexerSpeed(0);
        timer.stop();
        timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.Shooter.WAIT_UNTIL_END_SECS);
    }
}
