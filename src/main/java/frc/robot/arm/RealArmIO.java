package frc.robot.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class RealArmIO implements IArmIO {

  private CANSparkMax armMotorRight;
  private CANSparkMax armMotorLeft;
  private DutyCycleEncoder encoder;
  private DigitalInput homeSwitchLeft;
  private DigitalInput homeSwitchRight;
  private double motorPower;

  public RealArmIO() {
    armMotorRight = new CANSparkMax(Constants.Arm.ARM_MOTOR_RIGHT, MotorType.kBrushless);
    armMotorLeft = new CANSparkMax(Constants.Arm.ARM_MOTOR_LEFT, MotorType.kBrushless);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorRight.setInverted(true);
    armMotorLeft.setInverted(false);
    encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_CHANNEL);
    homeSwitchLeft = new DigitalInput(Constants.Arm.RIGHT_HOMESWITCH_CHANNEL);
    homeSwitchRight = new DigitalInput(Constants.Arm.LEFT_HOMESWITCH_CHANNEL);
  }

  @Override
  public void setMotorPower(double power) {
    // positive power makes arm move away from home
    motorPower = power;
    armMotorRight.setVoltage(power * 12);
    armMotorLeft.setVoltage(power * 12);
  }

  @Override
  public double getMeasurement() {
    return encoder.get() * 360;
  }

  @Override
  public double getMotorSpeed() {
    return motorPower;
  }

  @Override
  public boolean atFrontLimit() {
    return (getLeftHomeSwitch() || getRightHomeSwitch());
  }

  @Override
  public boolean getLeftHomeSwitch() {
    return !homeSwitchLeft.get();
  }

  @Override
  public boolean getRightHomeSwitch() {
    return !homeSwitchRight.get();
  }

  @Override
  public void periodic() {}
}
