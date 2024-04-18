package frc.robot.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SimShooterIO implements IShooterIO {
  private double shooterTopVoltage;
  private double shooterBottomVoltage;
  private double indexerVoltage;
  private final FlywheelSim topShooterMotorSim =
      new FlywheelSim(DCMotor.getNEO(1), Constants.Shooter.GEARING, Constants.Shooter.SHOOTER_MOI);
  private final FlywheelSim bottomShooterMotorSim =
      new FlywheelSim(DCMotor.getNEO(1), Constants.Shooter.GEARING, Constants.Shooter.SHOOTER_MOI);
  private final FlywheelSim indexerMotorSim =
      new FlywheelSim(DCMotor.getNEO(1), Constants.Shooter.GEARING, Constants.Shooter.INDEXER_MOI);

  public SimShooterIO() {}

  @Override
  public void setIndexerOutput(double power) {
    indexerVoltage = Math.min(1, Math.max(power, -1)) * 12;
  }

  @Override
  public void setShooterOutput(double topPower, double bottomPower) {
    shooterTopVoltage = Math.min(1, Math.max(topPower, -1)) * 12;
    shooterBottomVoltage = Math.min(1, Math.max(bottomPower, -1)) * 12;
  }

  @Override
  public double getMeasurement() {
    return 0;
  }

  @Override
  public boolean isIndexerLoaded() {
    return shooterTopVoltage > 0.001;
  }

  @Override
  public double[] getShooterSpeeds() {
    return new double[] {
      topShooterMotorSim.getAngularVelocityRPM(), bottomShooterMotorSim.getAngularVelocityRPM()
    };
  }

  @Override
  public double[] getIndexerSpeeds() {
    return new double[] {
      indexerMotorSim.getAngularVelocityRPM(), indexerMotorSim.getAngularVelocityRPM()
    };
  }

  @Override
  public void periodic() {
    topShooterMotorSim.setInputVoltage(shooterTopVoltage);
    topShooterMotorSim.update(0.02);

    bottomShooterMotorSim.setInputVoltage(shooterBottomVoltage);
    bottomShooterMotorSim.update(0.02);

    indexerMotorSim.setInputVoltage(indexerVoltage);
    indexerMotorSim.update(0.02);
  }
}
