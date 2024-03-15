package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class VoltageDashboard {
    private static boolean needsInit = true;
    public static void initVoltageDashboard() {
        if (needsInit) {
            ShuffleboardTab tab = Shuffleboard.getTab("VoltageInfo");
            tab.addDouble("Battery voltage", RobotController::getBatteryVoltage);
            tab.addDouble("6v Current", RobotController::getCurrent6V);
            tab.addDouble("5v Current", RobotController::getCurrent5V);
            tab.addDouble("3v Current", RobotController::getCurrent3V3);
            tab.addDouble("CAN bus usage %", () -> RobotController.getCANStatus().percentBusUtilization);

            needsInit = false;
        }
    }
}
