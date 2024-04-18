// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.undertaker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class UndertakerIntake extends Command {

    private final UndertakerSubsystem undertaker;
    private BooleanSupplier readyToIntake;

    public UndertakerIntake(UndertakerSubsystem undertaker, BooleanSupplier readyToIntake) {
        addRequirements(undertaker);
        this.undertaker = undertaker;
        this.readyToIntake = readyToIntake;
    }

    @Override
    public void initialize() {
        undertaker.run(Constants.Undertaker.INTAKE_SPEED);
    }

    @Override
    public void execute() {
        undertaker.run(readyToIntake.getAsBoolean() ? Constants.Undertaker.INTAKE_SPEED : 0);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
