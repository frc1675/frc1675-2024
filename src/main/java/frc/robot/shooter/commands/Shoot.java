// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;

public class Shoot extends Command {
    private ShooterSubsystem subsystem;
    private double targetSpeed;

    public Shoot(ShooterSubsystem subsystem, double targetSpeed) {
        this.subsystem = subsystem;
        this.targetSpeed = targetSpeed;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.setTargetIndexerSpeed(targetSpeed);
    }

    // Called every time the scheduler runs while the comm0and is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.setTargetShooterSpeed(0);
        subsystem.setTargetIndexerSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}