package undertaker;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.undertaker.TestUndertakerIO;
import frc.robot.undertaker.UndertakerSubsystem;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class UndertakerTest {

    private static UndertakerSubsystem undertaker;
    private static TestUndertakerIO undertakerIO;
    private static final double targetSpeed = 5;

    @BeforeAll
    public static void setup() {
        undertakerIO = new TestUndertakerIO();
        undertaker = new UndertakerSubsystem(undertakerIO);
    }

    @Test
    public void setsTargetSpeed() {
        undertaker.run(targetSpeed);
        assertTrue(undertakerIO.getDesiredSpeed() == targetSpeed);
    }

    @Test
    public void testRunMotorAlive() {
        boolean[] livingMotors = {true, true};
        undertakerIO.setIsAlive(livingMotors);
        undertaker.run(targetSpeed);
        assertTrue(undertakerIO.getHasRun());
    }

    @Test
    public void testRunMotorDead() {
        undertakerIO.resetHasRun();
        boolean[] deadMotor = {true, false};
        undertakerIO.setIsAlive(deadMotor);
        undertaker.run(targetSpeed);
        assertFalse(undertakerIO.getHasRun());
    }
}
