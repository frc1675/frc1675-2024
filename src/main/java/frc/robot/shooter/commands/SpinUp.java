// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.shooter.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.shooter.ShooterSubsystem;

public class SpinUp extends Command {
    private final ShooterSubsystem subsystem;
    private final BooleanSupplier slowShoot;

    public SpinUp(ShooterSubsystem subsystem, BooleanSupplier slowShoot) {
        this.subsystem = subsystem;
        this.slowShoot = slowShoot;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        double scale = slowShoot.getAsBoolean() ? Constants.Shooter.AMP_SHOOT_SCALE : 1;

        subsystem.setTargetShooterSpeed(Constants.Shooter.SHOOT_SPEED * scale);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return subsystem.isShooterReady();
    }
}
