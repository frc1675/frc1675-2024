package frc.robot.subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.testing.SimSparkMax;
import frc.robot.util.testing.SparkMax;
import frc.robot.util.testing.TestableSparkMax;

public class ExampleSubsystem extends SubsystemBase implements AutoCloseable {

    private TestableSparkMax motor;
    private Solenoid solenoid;
    
    public ExampleSubsystem() {
        if(Robot.isReal()) {
            motor = new SparkMax(new CANSparkMax(0, MotorType.kBrushless));
        }else {
            motor = new SimSparkMax();
        }

        solenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);

    }

    public void extend() {
        solenoid.set(true);
        motor.setSpeed(0);
    }

    public void retract() {
        solenoid.set(false);
    }

    public void setSpeed(double speed) {
        if(!solenoid.get()) {
            motor.setSpeed(speed);
        }
    }

    public double getSpeed() {
        return motor.getSpeed();
    }

    @Override
    public void close() throws Exception {
        solenoid.set(false);
        solenoid.close();
        motor.close();
    }

}
