package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.poseScheduler.FieldArea2d;

public class Constants {

    public class Drive {
        public static final double MAXIMUM_VELOCITY = 5.0; // meters per second
        public static final double MAXIMUM_ANGULAR_VELOCITY = 10; // radians per second

        public static final double AUTONOMOUS_VELOCITY = MAXIMUM_VELOCITY; //meters per second
        public static final double AUTONOMOUS_ACCELERATION = 10.0; // meters per second squared

        public static final double MAXIMUM_VISION_POSE_OVERRIDE_DISTANCE = 1.0; // meters

        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double STEER_GEAR_RATIO = 12.8;
        public static final double PULSE_PER_ROTATION = 1;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

        public static final double SLOW_DRIVE_SCALE = 0.5;
    }

    public class Arm {
        // Encoder constants
        public static final int ENCODER_CHANNEL = 7;
        public static final int ENCODER_CHANNEL_B = 29;
        public static final int ENCODER_COUNT = 1024;
        // Home switch constants
        public static final int RIGHT_HOMESWITCH_CHANNEL = 8;
        public static final int LEFT_HOMESWITCH_CHANNEL = 9;
        // Motor constants
        public static final int ARM_MOTOR_RIGHT = 15;
        public static final int ARM_MOTOR_LEFT = 16;
        // PID constants
        public static final double PID_P_COEFFICIENT = 0.023;
        public static final double PID_I_COEFFICIENT = 0;
        public static final double PID_D_COEFFICIENT = 0;
        // Arm movment lmits constants
        public static final double TARGET_RANGE = 2.0;
        public static final double DEBOUNCE_TIME = 0.5;
        // Position constnats
        public static final double HOME_POSITION = 138;
        public static final double HIGH_SCORE_POSITION = 60;
        public static final double AMP_POSITION = 35;
        public static final double MAX_ARM_RANGE_DEGREES = 20; // 13 degrees is vertical
        // PID Profile constants
        public static final double MAXIMUM_VELOCITY = 150; // degrees per second
        public static final double MAXIMUM_ACCELERATION = 750; // degrees per second squared

        public static final double LONG_SHOT_ANGLE = 105;
    }

    public class PathPlanner {
        public static final boolean PATH_PLANNER_IS_ENABLED = true;

        public static final double MAXIMUM_VELOCITY = 2.5; // meters per second
        public static final double MAXIMUM_ACCELERATION = 1.25; // meters per second squared

        public static final double MAXIMUM_ANGULAR_VELOCITY = 7.5; // radians per second
        public static final double MAXIMUM_ANGULAR_ACCELERATION = 3.75; // radians per second squared

        public static final double DRIVEBASE_RADIUS = Units.inchesToMeters(14.5); // meters

        public static final double TRANSLATION_P = 7;
        public static final double TRANSLATION_I = 0;
        public static final double TRANSLATION_D = 0;

        public static final double ROTATION_P = 0.000;
        public static final double ROTATION_I = 0;
        public static final double ROTATION_D = 0;

        public static final double DYNAMIC_PATHING_MAX_DISTANCE = 5; // meters
    }

    public class Field {
        public static final Translation2d SPEAKER_SCORING_POSITION = new Translation2d(1.67, 5.52);

        public static final FieldArea2d FRIENDLY_ALLIANCE_AREA = new FieldArea2d(0, 0, 5.85, 8.21);

        public static final Pose2d SUBWOOFER_FRONT = new Pose2d(1.35, 5.55, Rotation2d.fromDegrees(0)); 
        
        //Relative to the drivers
        public static final Pose2d SUBWOOFER_LEFT = new Pose2d(0.75, 6.7, Rotation2d.fromDegrees(60));
        public static final Pose2d SUBWOOFER_RIGHT = new Pose2d(0.75, 4.35, Rotation2d.fromDegrees(-60));
    }

    public class Shooter {
        public static final double SHOOT_SPEED = 1700;
        public static final double INTAKE_SPEED = Undertaker.INTAKE_SPEED * .09;

        public static final int INDEXER_MOTOR_TOP = 18;
        public static final int INDEXER_MOTOR_BOTTOM = 19;

        public static final int SHOOTER_MOTOR_TOP = 17; // might be wrong
        public static final int SHOOTER_MOTOR_BOTTOM = 20;

        public static final int LASER_CAN = 21;
        public static final int INDEXER_NOTE_DETECTION_RANGE = 150; // PLACEHOLDER
        public static final double TARGET_SPEED_ERROR_MARGIN = 75; // PLACEHOLDER

        public static final double SHOOTER_PID_P = 0.0002;
        public static final double SHOOTER_PID_I = 0.0001;
        public static final double SHOOTER_PID_D = 0;

        public static final double SHOOTER_FF_V = 0.00015;
        public static final double SHOOTER_FF_S = 0;

        public static final double SHOOTER_SHOOT_TIME = 0.5;

        public static final String SHUFFLEBOARD_TAB = "Shooter";
        public static final double GEARING = 0.5;

        public static final double SHOOTER_MOI = .001; // Joules * kg / m^2
        public static final double INDEXER_MOI = 1;

        public static final double AMP_SHOOT_SCALE = 0.4;
        public static final double LONG_SHOT_SPEED = 5000;
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

        public static final double INTAKE_SPEED = 0.8;

        public static final double EJECT_SPEED = -0.5;

    }
}
