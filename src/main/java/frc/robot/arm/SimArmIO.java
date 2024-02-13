package frc.robot.arm;

import edu.wpi.first.math.VecBuilder;
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

    private final Mechanism2d mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d arm = 
        armPivot.append( 
            new MechanismLigament2d(
                "Arm",
                30,
                Units.radiansToDegrees(armSim.getAngleRads()),
                6,
                new Color8Bit(Color.kYellow))
            );


    public SimArmIO(){
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
    public double getMeasurement(){
        return Units.radiansToDegrees(angleRads);
    }

    @Override
    public double getMotorSpeed(){
        return motorSpeed;
    }

    @Override
    public boolean atFrontLimit(){
        return homeSwitch;
    }

    @Override
    public void periodic(){
        armSim.setInput(motorSpeed *12);
        armSim.update(0.020);
        angleRads = armSim.getAngleRads();
        arm.setAngle(Units.radiansToDegrees(angleRads));
    }
}
