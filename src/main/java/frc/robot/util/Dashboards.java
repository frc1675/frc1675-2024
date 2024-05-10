package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.generated.git.BuildGitInfo;

public class Dashboards {
    private static boolean voltageNeedsInit = true;
    private static boolean currentNeedsInit = true;
    private static boolean memoryNeedsInit = true;
    private static boolean gitInfoNeedsInit = true;

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
            tab.addDouble("Shooter Motor Top Current", () -> PDH.getCurrent(12));
            tab.addDouble("Shooter Motor Bottom Current", () -> PDH.getCurrent(7));
            tab.addDouble("Indexer Motor Top Current", () -> PDH.getCurrent(13));
            tab.addDouble("Indexer Motor Bottom Current", () -> PDH.getCurrent(6));
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
            memoryNeedsInit = false;
        }
    }

    public static void initGitInfoDashboard() {
        if (gitInfoNeedsInit) {
            ShuffleboardTab tab = Shuffleboard.getTab("Git Info");
            tab.addString("Branch", () -> BuildGitInfo.GIT_BRANCH);
            tab.addString("Commit", () -> BuildGitInfo.GIT_SHA.substring(0, 8));
            tab.addBoolean("Dirty?", () -> BuildGitInfo.DIRTY != 0); // true if any uncommitted changes
            tab.addString("Build Time", () -> BuildGitInfo.BUILD_DATE);
            gitInfoNeedsInit = false;
        }
    }

}
