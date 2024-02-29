// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.undertaker;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class UndertakerIntake extends Command {

    private final UndertakerSubsystem undertaker;
    private BooleanSupplier isIndexerLoaded;

    public UndertakerIntake(UndertakerSubsystem undertaker, BooleanSupplier isIndexerLoaded) {
        addRequirements(undertaker);
        this.undertaker = undertaker;
        this.isIndexerLoaded = isIndexerLoaded;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        undertaker.run(Constants.Undertaker.INTAKE_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        undertaker.run(isIndexerLoaded.getAsBoolean() ? 0 : Constants.Undertaker.INTAKE_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
