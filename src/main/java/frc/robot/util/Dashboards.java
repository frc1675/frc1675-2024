package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboards {
  private static boolean voltageNeedsInit = true;
  private static boolean memoryNeedsInit = true;

  public static void initVoltageDashboard() {
    if (voltageNeedsInit) {
      ShuffleboardTab tab = Shuffleboard.getTab("VoltageInfo");
      tab.addDouble("Battery voltage", RobotController::getBatteryVoltage);
      tab.addDouble("6v Current", RobotController::getCurrent6V);
      tab.addDouble("5v Current", RobotController::getCurrent5V);
      tab.addDouble("3v Current", RobotController::getCurrent3V3);
      tab.addDouble("CAN bus usage %", () -> RobotController.getCANStatus().percentBusUtilization);

      voltageNeedsInit = false;
    }
  }

  public static void initMemoryDashboard() {
    if (memoryNeedsInit) {
      Runtime r = Runtime.getRuntime();
      Shuffleboard.getTab("Memory Usage")
          .addDouble("Usage percent", () -> r.freeMemory() / (double) r.totalMemory());
    }
  }
}
