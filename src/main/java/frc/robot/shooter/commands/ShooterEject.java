// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;

public class ShooterEject extends Command {
    private final ShooterSubsystem subsystem;
    private final double targetSpeed;

    public ShooterEject(ShooterSubsystem subsystem, double targetSpeed) {
        this.subsystem = subsystem;
        this.targetSpeed = targetSpeed;  
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (subsystem.isIndexerLoaded()) {
            subsystem.setIndexerSpeed(-Math.abs(targetSpeed));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.setIndexerSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
