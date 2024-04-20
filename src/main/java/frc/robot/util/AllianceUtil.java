package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {

    public static boolean isRedAlliance() {
        if (DriverStation.getAlliance().isEmpty()) {
            return false;
        }

        return DriverStation.getAlliance().get().equals(Alliance.Red);
    }

    public static int getTranslationDirection() {
        return isRedAlliance() ? 1 : -1;
    }
}
