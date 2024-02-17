import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.TestArmIO;
import jdk.jfr.Timestamp;
import frc.robot.Constants;

public class ArmTest {

  private static ArmSubsystem arm;
  private static TestArmIO armIO;

  @BeforeAll
  public static void setup() {
    armIO = new TestArmIO();
    arm = new ArmSubsystem(armIO);
  }

  @Test
  public void shouldBreakProperly() {
    armIO.setHomeSwitch(true);
    armIO.setAngleMeasurement(Constants.Arm.TARGET_RANGE+1);
    arm.periodic();
    assertTrue(arm.isBroken());
  }

  @Test
  public void homeswitchShouldStopIfWrongWay(){
    armIO.setHomeSwitch(true);
    armIO.setMotorPower(-1);
    arm.periodic();
    assertTrue(armIO.getMotorSpeed() == 0);
  }

  @Test 
  public void shouldStopIfWrongWay(){
    armIO.setAngleMeasurement(Constants.Arm.MAX_ARM_RANGE_DEGREES+1);
    arm.periodic();
    assertTrue(armIO.getMotorSpeed() == 0);
  }

  public void operateAsNormal(){
    arm.setTarget(Constants.Arm.MAX_ARM_RANGE_DEGREES);
    armIO.setAngleMeasurement(Constants.Arm.HIGH_SCORE_POSITION);
    arm.periodic();
    assertTrue(armIO.getMotorSpeed() > 0);
  }

}
