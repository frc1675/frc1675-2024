package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

public class Constants {

  public class Drive {
    public static final double MAXIMUM_VELOCITY = 5.5; // meters per second
    public static final double MAXIMUM_ANGULAR_VELOCITY = 8; // radians per second

    public static final double AUTONOMOUS_VELOCITY = MAXIMUM_VELOCITY; // meters per second
    public static final double AUTONOMOUS_ACCELERATION = 10.0; // meters per second squared

    public static final double MAXIMUM_VISION_POSE_OVERRIDE_DISTANCE = 1.0; // meters

    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double PULSE_PER_ROTATION = 1;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

    public static final double SLOW_DRIVE_SCALE = 0.5;

    public static final double ROTATION_P = 0.05;
    public static final double ROTATION_I = 0.001;
    public static final double ROTATION_D = 0;
    public static final double ROTATION_TARGET_RANGE = 1.5;
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
    public static final double DEBOUNCE_TIME = 0.25;
    // Position constnats
    public static final double HOME_POSITION = 138;
    public static final double PODIUM_SHOT_ANGLE = HOME_POSITION - 33;
    public static final double BEHIND_NOTE_B_ANGLE = Auto.BEHIND_CLOSE_B_SHOT_ANGLE;

    public static final double HIGH_SCORE_POSITION = HOME_POSITION - 78;
    public static final double AMP_POSITION = HOME_POSITION - 103;
    public static final double MAX_ARM_RANGE_DEGREES =
        HOME_POSITION - 118; // home - 125 degrees is vertical
    // PID Profile constants
    public static final double MAXIMUM_VELOCITY = 150; // degrees per second
    public static final double MAXIMUM_ACCELERATION = 750; // degrees per second squared
  }

  public class Auto {
    public static final double MODULE_MAXIMUM_VELOCITY = 5.5; // meters per second

    public static final double DRIVEBASE_RADIUS = Units.inchesToMeters(14.5); // meters

    public static final double TRANSLATION_P = 5;
    public static final double TRANSLATION_I = 0;
    public static final double TRANSLATION_D = 0;

    public static final double ROTATION_P = 5;
    public static final double ROTATION_I = 0;
    public static final double ROTATION_D = 0;

    public static final double DYNAMIC_PATHING_MAX_DISTANCE = 5; // meters

    public static final double CLOSE_SHOT_SPEED_TOP = 1700;
    public static final double CLOSE_SHOT_SPEED_BOTTOM = CLOSE_SHOT_SPEED_TOP * 0.9;
    public static final double SHOT_SPEED = 3500;
    public static final double SHOOT_TIME = 0.25;
    public static final double SHOOTER_INTAKE_SPEED = Undertaker.INTAKE_SPEED * 0.09;

    public static final double UNDERTAKER_INTAKE_SPEED = Undertaker.INTAKE_SPEED;

    public static final double INTAKE_ATTEMPT_TIMEOUT = 0.5;

    public static final double CLOSE_A_SHOT_ANGLE = Arm.HOME_POSITION - 35;
    public static final double CLOSE_B_SHOT_ANGLE = Arm.HOME_POSITION - 31;
    public static final double CLOSE_C_SHOT_ANGLE = Arm.HOME_POSITION - 34;
    public static final double CLOSER_C_SHOT_ANGLE = Arm.HOME_POSITION - 25;
    public static final double BEHIND_CLOSE_B_SHOT_ANGLE = Arm.HOME_POSITION - 38;
    public static final double SOURCE_SIDE_SHOT_ANGLE = Arm.HOME_POSITION - 39;

    public static final double FAR_SHOT_ANGLE = CLOSE_B_SHOT_ANGLE; // TODO measure this

    public static final double AUTO_DEBOUNCE_TIME = 0.1;

    public static final double SUFFICIENT_EXTRA_PATHFINDING_TIME = 2;
    public static final PathConstraints EXTRA_PATHFINDING_CONSTRAINTS =
        new PathConstraints(3, 3, 9.4, 12.5);
  }

  public class Shooter {
    public static final double SHOOT_SPEED = 1700;
    public static final double AMP_SHOOT_SPEED = SHOOT_SPEED * 0.4;
    public static final double LONG_SHOT_SPEED = 3500;

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

    public static final double SHOOTER_FF_V = 0.00012;
    public static final double SHOOTER_FF_S = 0;

    public static final double SHOOTER_SHOOT_TIME = 0.5;

    public static final String SHUFFLEBOARD_TAB = "Shooter";
    public static final double GEARING = 0.5;

    public static final double SHOOTER_MOI = .001; // Joules * kg / m^2
    public static final double INDEXER_MOI = 1;
  }

  public class Dashboard {
    public static final boolean DISABLE_TUNER = false;
    public static final String VERSION_FILE_NAME = "version/.robotVersionMini";
  }

  public class Controller {
    public static final double DEADZONE_CONSTANT = 0.1675;

    public static final double RUMBLE_POWER = 1;
    public static final double RUMBLE_TIME = 0.5;

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
