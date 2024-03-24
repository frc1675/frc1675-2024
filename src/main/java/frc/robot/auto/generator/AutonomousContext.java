package frc.robot.auto.generator;

import java.util.ArrayList;
import java.util.List;

public class AutonomousContext {
    
    private final List<Double> armAngles;
    private int index = -1;

    private final double defaultAngle;

    public AutonomousContext(double defaultAngle) {
        armAngles = new ArrayList<Double>();
        this.defaultAngle = defaultAngle;
    }

    public double getAndIncrement() {
        if (armAngles.size() >= index + 1) {
            return defaultAngle;
        }
        index++;
        return armAngles.get(index);
    }

    public void clear() {
        armAngles.clear();
    }

    public void addAngle(double angle, double...others) {
        armAngles.add(angle);
        for (double d : others) {
            armAngles.add(d);
        }
    }

}
