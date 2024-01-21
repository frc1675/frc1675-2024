import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import frc.robot.subsystem.ExampleSubsystem;

public class ExampleTest {
    
    static final double DELTA = 1e-2;

    ExampleSubsystem sub;
    SolenoidSim simSolenoid;
    
    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);

        sub = new ExampleSubsystem();
        simSolenoid = new SolenoidSim(PneumaticsModuleType.REVPH, 1);
    }

    @AfterEach
    void reset() throws Exception {
        sub.close();
    }

    @Test 
    void startingStateIsCorrect() {
        assertFalse(simSolenoid.getOutput());
        assertEquals(0, sub.getSpeed());
    }

    @Test 
    void solenoidToggles() {
        sub.extend();
        assertTrue(simSolenoid.getOutput());

        sub.retract();
        assertFalse(simSolenoid.getOutput());
    }

    @Test 
    void motorSetSpeedIsCorrect() {
        sub.setSpeed(1);
        assertEquals(1, sub.getSpeed());
    }

    @Test 
    void motorDoesntMoveWhileExtended() {
        sub.extend();
        sub.setSpeed(1);
        assertEquals(0, sub.getSpeed(), DELTA);

        sub.retract();
        assertEquals(0, sub.getSpeed(), DELTA);

        sub.setSpeed(1);
        assertEquals(1, sub.getSpeed(), DELTA); 

        sub.extend();
        assertEquals(0, sub.getSpeed(), DELTA);
    }
}
