package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboards {
    private static boolean voltageNeedsInit = true;
    private static boolean currentNeedsInit = true;
    private static boolean memoryNeedsInit = true;
    private static final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

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

    public static void initCurrentDashboard() {
        if (currentNeedsInit) {
            ShuffleboardTab tab = Shuffleboard.getTab("Current Info");
            // Arm Motors
            tab.addDouble("Right Arm Motor Current", () -> PDH.getCurrent(18));
            tab.addDouble("Left Arm Motor Current", () -> PDH.getCurrent(5));
            // Shooter Motors
            tab.addDouble("Shooter Motor 1 Current", () -> PDH.getCurrent(6));
            tab.addDouble("Shooter Motor 2 Current", () -> PDH.getCurrent(7));
            tab.addDouble("Shooter Motor 3 Current", () -> PDH.getCurrent(12));
            tab.addDouble("Shooter Motor 4 Current", () -> PDH.getCurrent(13));
            // Undertaker Motors
            tab.addDouble("Undertaker left Motor Current", () -> PDH.getCurrent(4));

            // Drive Motors

            currentNeedsInit = false;
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
