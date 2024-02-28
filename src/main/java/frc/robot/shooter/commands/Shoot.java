// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;

public class Shoot extends Command {
    private ShooterSubsystem subsystem;

    public Shoot(ShooterSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setIndexerSpeed(1);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setIndexerSpeed(0);
        // spin down shooter
        subsystem.setTargetShooterSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
