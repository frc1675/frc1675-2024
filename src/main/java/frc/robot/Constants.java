package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public class Drive {
        public static final double MAXIMUM_VELOCITY = 5.0; //meters per second
        public static final double MAXIMUM_ANGULAR_VELOCITY = 10; //radians per second

        public static final double MAXIMUM_VISION_POSE_OVERRIDE_DISTANCE = 1.0; //meters

        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double STEER_GEAR_RATIO = 12.8;
        public static final double PULSE_PER_ROTATION = 1;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

    }

    public class PathPlanner {
        //TODO tune constants if necessary
        public static final double MAXIMUM_VELOCITY= 2.5; //meters per second
        public static final double MAXIMUM_ACCELERATION = 1.25; //meters per second squared

        public static final double MAXIMUM_ANGULAR_VELOCITY = 7.5; //radians per second
        public static final double MAXIMUM_ANGULAR_ACCELERATION = 3.75; //radians per second squared

        //TODO measure on robot
        public static final double DRIVEBASE_RADIUS = Units.inchesToMeters(13.7885); //meters

        public static final double TRANSLATION_P = 1000;

        public static final double ROTATION_P = 1;
        
    }

    public class Dashboard {
        public static final boolean DISABLE_TUNER = false;
        public static final String VERSION_FILE_NAME = "version/.robotVersionMini";
    }

    public class Controller {
        public static final double DEADZONE_CONSTANT = 0.1675;

        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;
        public static final int LEFT_X_AXIS = 0;
        public static final int LEFT_Y_AXIS = 1;
        public static final int RIGHT_X_AXIS = 4;
        public static final int RIGHT_Y_AXIS = 5;
        public static final int RIGHT_TRIGGER = 3;

        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;

        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int BACK_BUTTON = 7;
        public static final int START_BUTTON = 8;
        public static final int LEFT_JOYSTICK_BUTTON = 9;
        public static final int RIGHT_JOYSTICK_BUTTON = 10;
    }

    public class Undertaker {
        public static final int INTAKE_MOTOR_ONE = 13;
        public static final int INTAKE_MOTOR_TWO = 14;

        public static final double INTAKE_SPEED = 0.5;
        public static final double EJECT_SPEED = -0.5;

    }    
}
