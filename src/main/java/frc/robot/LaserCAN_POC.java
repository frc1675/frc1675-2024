package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;

public class LaserCAN_POC extends SubsystemBase {
    private LaserCan laserCAN;

    public LaserCAN_POC(){
        laserCAN = new LaserCan(0);
    }

    public  int getMeasurementmm(){
        LaserCan.Measurement measurement = laserCAN.getMeasurement();
        if(measurement != null){
            return measurement.distance_mm;
        }
        return -1;
    }
}