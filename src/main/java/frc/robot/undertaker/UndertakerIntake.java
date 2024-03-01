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

    @Override
    public void initialize() {
        undertaker.run(Constants.Undertaker.INTAKE_SPEED);
    }

    @Override
    public void execute() {
        undertaker.run(isIndexerLoaded.getAsBoolean() ? 0 : Constants.Undertaker.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
