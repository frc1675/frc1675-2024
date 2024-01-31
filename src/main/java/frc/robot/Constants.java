package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public class Drive {
        public static final double MAXIMUM_VELOCITY = 5.5; //meters per second
        public static final double MAXIMUM_ANGULAR_VELOCITY = 15.5; //radians per second

        public static final double MAXIMUM_VISON_POSE_OVERRIDE_DISTANCE = 1.0; //meters

        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double STEER_GEAR_RATIO = 12.8;
        public static final double PULSE_PER_ROTATION = 42; //encoder resolution
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

    }
    public class Arm{
        public static final int ENCODER_CHANNEL_A = 28;
        public static final int ENCODER_CHANNEL_B = 29;
        public static final int ENCODER_COUNT = 8192;
        public static final int ARM_MOTOR_ONE = 11;
        public static final int ARM_MOTOR_TWO = 12;
        public static final double HOME_POSITION = 0;
        public static final double P_COEFFICIENT = 1;
        public static final double I_COEFFICIENT = 0;
        public static final double D_COEFFICIENT = 0;
        public static final double HOME_POSITION_RANGE_DEGREES = 5.0;
        public static final double TARGET_RANGE_DEGREES = 5.0;
        public static final int DIGITAL_INPUT_CHANNEL = 28;
        public static final double ARM_ANGLE_LIMIT_DEGREES = 0;
        public static final double HOME_SWITCH_FAILSAFE_DEGREES = 0;
    }
   
    public class PathPlanner {
        //TODO tune constants if necessary
        public static final double MAXIMUM_VELOCITY= 5.0; //meters per second
        public static final double MAXIMUM_ACCELERATON = 2.0; //meters per second squared

        public static final double MAXIMUM_ANGULAR_VELOCITY = 15.0; //radians per second
        public static final double MAXIMUM_ANGULAR_ACCELERATION = 5.0; //radians per second squared

        //TODO measure on robot
        public static final double DRIVEBASE_RADIUS = Units.inchesToMeters(13.7885); //meters
        
    }

    public class Dashboard {
        public static final boolean DISABLE_TUNER = false;
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
}
