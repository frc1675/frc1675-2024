package frc.robot.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class SimArmIO implements IArmIO{
    private static final double TICK = 2.0 * Math.PI / Constants.Arm.ENCODER_COUNT;

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getNEO(2),
        100,
        SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 13), 
        Units.inchesToMeters(30), 
        Units.degreesToRadians(Constants.Arm.HOME_POSITION), 
        Units.degreesToRadians(200), 
        true, 
        Units.degreesToRadians(Constants.Arm.HOME_POSITION), 
        VecBuilder.fill(TICK)
    );

    private double angleRads;
    private double motorSpeed;
    private boolean homeSwitch;

    public SimArmIO(){
        homeSwitch = false;
    }

    @Override
    public void setMotorPower(double power) {
        motorSpeed = power;
    }

    @Override
    public double getMeasurement(){
        return Units.radiansToDegrees(angleRads);
    }

    @Override
    public boolean atFrontLimit(){
        return homeSwitch;
    }

    @Override
    public void periodic(){
        armSim.setInput(motorSpeed*12);
        armSim.update(0.020);
        angleRads = armSim.getAngleRads();
    }
}
