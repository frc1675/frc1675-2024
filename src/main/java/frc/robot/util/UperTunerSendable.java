package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class UperTunerSendable implements Sendable {

  private final double defaultValue;
  private double currentValue;

  private final double max;
  private final double min;

  public UperTunerSendable(double defaultValue) {
    this.defaultValue = defaultValue;
    this.currentValue = defaultValue;
    this.max = 100.0;
    this.min = 0.0;
  }

  public UperTunerSendable(double defaultValue, double min, double max) {
    this.defaultValue = defaultValue;
    this.currentValue = defaultValue;
    this.max = max;
    this.min = min;
  }

  public double getDefaultValue() {
    return defaultValue;
  }

  public double getCurrentValue() {
    // disable tuner if is attached to a FMS or explicitly disabled
    if (Constants.Dashboard.DISABLE_TUNER || DriverStation.isFMSAttached()) {
      return defaultValue;
    }
    return currentValue;
  }

  public double getMax() {
    return max;
  }

  public double getMin() {
    return min;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("UperTuner");
    builder.addDoubleProperty("defaultValue", () -> defaultValue, null);
    builder.addDoubleProperty(
        "currentValue",
        () -> currentValue,
        (double v) -> {
          currentValue = v;
        });
    builder.addDoubleProperty("max", () -> max, null);
    builder.addDoubleProperty("min", () -> min, null);
  }
}
