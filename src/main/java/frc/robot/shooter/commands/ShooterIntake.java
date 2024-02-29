// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class ShooterIntake extends Command {
    private final ShooterSubsystem subsystem;

    public ShooterIntake(ShooterSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setIndexerSpeed(Constants.Shooter.INTAKE_SPEED);
    }

    @Override
    public void execute() {
    	subsystem.setIndexerSpeed(subsystem.isIndexerLoaded() ? 0: Constants.Shooter.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        // subsystem.setIndexerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
