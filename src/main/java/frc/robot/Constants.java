package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.poseScheduler.FieldArea2d;

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
        public static final double DRIVEBASE_RADIUS = Units.inchesToMeters(14.5); //meters

        public static final double TRANSLATION_P = 1000;
        public static final double TRANSLATION_I = 0;
        public static final double TRANSLATION_D = 0;

        public static final double ROTATION_P = 1;
        public static final double ROTATION_I = 0;
        public static final double ROTATION_D = 0;

        public static final double DYNAMIC_PATHING_MAX_DISTANCE = 5; //meters
    }

    public class Field {
        public static final Translation2d SPEAKER_SCORING_POSITION = new Translation2d(1.67, 5.52);

        public static final FieldArea2d FRIENDLY_ALLIANCE_AREA = new FieldArea2d(0, 0, 5.85, 8.21);
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
    }

    public class LEDController {
        public static final int LED_CONTROLLER_CHANNEL = 0;
    }

    public class Undertaker {
        public static final int INTAKE_MOTOR_ONE = 13;
        public static final int INTAKE_MOTOR_TWO = 14;

        public static final double INTAKE_SPEED = 0.5;
        public static final double EJECT_SPEED = -0.5;

    }    
}
