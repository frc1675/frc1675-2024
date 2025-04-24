package frc.robot.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class SimArmIO implements IArmIO {
    private static final double TICK = 2.0 * Math.PI / Constants.Arm.ENCODER_COUNT;
    private static final double SIM_HOME = -32.5;
    // arm sim object has angle -90 straight down, 0 straight ahead, angle goes up
    // as arm goes up
    // Sim domain:home -32.5, vertical 90, amp 100
    // robot domain: home 138, vertical 15.5, amp 5.5
    // robot to sim: 105.5 - r_angle = sim_angle
    // sim to robot: 105.5 - sim_angle = r_angle
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            100.0,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 13),
            Units.inchesToMeters(30),
            Units.degreesToRadians(domainSwap(Constants.Arm.HOME_POSITION)),
            Units.degreesToRadians(domainSwap(Constants.Arm.MAX_ARM_RANGE_DEGREES)),
            true,
            Units.degreesToRadians(domainSwap(Constants.Arm.HOME_POSITION)));

    private double angleRads;
    private double motorSpeed;
    private boolean homeSwitch;

    private final Mechanism2d mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d arm = armPivot.append(new MechanismLigament2d(
            "Arm", 30, domainSwap(Constants.Arm.HOME_POSITION), 6, new Color8Bit(Color.kYellow)));

    public SimArmIO() {
        homeSwitch = false;
        ShuffleboardTab tab = Shuffleboard.getTab("Arm Display");
        tab.add("Arm Graphic", mech2d);
        armTower.setColor(new Color8Bit(Color.kBlue));
    }

    @Override
    public void setMotorPower(double power) {
        motorSpeed = power;
    }

    @Override
    public double getMeasurement() {
        return domainSwap(Units.radiansToDegrees(angleRads));
    }

    @Override
    public double getMotorSpeed() {
        return motorSpeed;
    }

    @Override
    public boolean atFrontLimit() {
        return homeSwitch;
    }

    @Override
    public void periodic() {
        armSim.setInput(motorSpeed * 12);
        armSim.update(0.020);
        angleRads = armSim.getAngleRads();
        arm.setAngle(Units.radiansToDegrees(angleRads));
    }

    private double domainSwap(double angleDeg) {
        return Constants.Arm.HOME_POSITION + SIM_HOME - angleDeg;
    }

    @Override
    public boolean getRightHomeSwitch() {
        return false;
    }

    @Override
    public boolean getLeftHomeSwitch() {
        return false;
    }
}
