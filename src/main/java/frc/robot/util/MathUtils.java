package frc.robot.util;

import frc.robot.Constants;

public class MathUtils {
    
    public static double getDeadzoneAdjustedInput(double raw) {   
        if(Math.abs(raw) > Constants.Controller.DEADZONE_CONSTANT) {
            if(raw > 0) {
                return (raw - Constants.Controller.DEADZONE_CONSTANT) / (1 - Constants.Controller.DEADZONE_CONSTANT);
            }
            return (raw + Constants.Controller.DEADZONE_CONSTANT) / (1 - Constants.Controller.DEADZONE_CONSTANT);
        }
        return 0;
    }
}
