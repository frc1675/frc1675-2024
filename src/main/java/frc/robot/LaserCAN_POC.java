package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;

public class LaserCAN_POC extends SubsystemBase {
    private LaserCan laserCAN;

    public LaserCAN_POC(){
        laserCAN = new LaserCan(0);
        Shuffleboard.getTab("LaserCAN").addInteger("Object Distance", () -> getMeasurementmm());
    }

    public int getMeasurementmm(){
        Measurement measurement = laserCAN.getMeasurement();
        if(measurement != null){
            return measurement.distance_mm;
        }
        return -1;
    }

    
}