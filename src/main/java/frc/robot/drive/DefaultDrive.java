// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDrive extends Command {
  
  private DriveSubsystem drive;
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier rotation;

  public DefaultDrive(DriveSubsystem drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
    this.drive = drive;
    this.x = x;
    this.y = y;
    this.rotation = rotation;

    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drive.drive(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
