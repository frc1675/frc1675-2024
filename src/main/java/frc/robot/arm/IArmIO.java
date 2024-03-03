package frc.robot.arm;

public interface IArmIO {

    // provides power value from -1.0 to 1.0. Positive power drives arm UP
    public void setMotorPower(double power);

    // returns arm angle in degrees, angle goes DOWN in value as arm goes UP
    public double getMeasurement();

    // returns the speed of the motors from -1.0 to 1.0
    public double getMotorSpeed();

    // returns whether or not the home switch is hit
    public boolean atFrontLimit();

    public boolean getRightHomeSwitch();

    public boolean getLeftHomeSwitch();

    // periodic loop to keep IO logic up to date
    public void periodic();
}