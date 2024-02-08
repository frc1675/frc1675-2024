package frc.robot.arm;

public interface IArmIO {

    //provides power value from -1.0 to 1.0
    public void setMotorPower(double power);

    //returns arm angle in degrees, -90 is straight down and 0 is horizontal forwards
    public double getMeasurement();

    //returns whether or not the home switch is hit
    public boolean atFrontLimit();

    //periodic loop to keep IO logic up to date
    public void periodic();
}