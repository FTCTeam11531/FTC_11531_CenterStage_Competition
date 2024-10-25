package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * <h2>Robot Constants</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Robot Constant values and settings
 * All-in-one-place
 * </p>
 * <br>
 */
public class utilRobotConstants {

    // General 'Global' Constants
    // ---------------------------------------

    // ---------------------------------------
    // Conversion Factor(s)
    // ---------------------------------------
    // Conversion Millimeters and Inches
    // - - Divide Millimeters by Conversion to get Inches
    // - - Multiply Inches by Conversion to get mm
    public static final double CONVERSION_FACTOR_MM_INCH = 25.4;

    // Conversion Radians and Degrees
    // - - Divide Degrees by Conversion to get Radians
    // - - Multiply Radians by Conversion to get Degrees
    public static final double CONVERSION_FACTOR_RADIAN_DEGREE = 57.2957795;

    /**
     *  <h2>Robot Constant Values - About The Robot</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Defines 'About the Team' robot constant settings and properties.
     * </p>
     * <li>Program Team</li>
     * <li>Development Season</li>
     * <br>
     */
    public static final class About {
        // Document Comments and any other 'About the Program' reference
        public static final String COMMENT_AUTHOR_NAME = "FTC Team 11531 - Green Team";
        public static final String COMMENT_SEASON_PERIOD = "2023 / 2024";
    }

    /**
     *  <h2>Robot Constant Values - Configuration</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Robot configuration constant settings and properties.
     * </p>
     * <li>Hardware Labels</li>
     * <br>
     */
    public static final class Configuration {
        // Hardware Configuration from Control Hub and/or Expansion Hub
        // ------------------------------------------------------------

        // Drivetrain
        public static final String LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT = "left_front_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_LEFT_BACK = "left_back_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT = "right_front_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK = "right_back_drive";

        public static final String LABEL_CONTROLHUB_IMU = "imu_ch";
        public static final String LABEL_EXPANSIONHUB_IMU = "imu_eh";

        public static final String LABEL_DRIVETRAIN_ENCODER_LEFT = LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT;
        public static final String LABEL_DRIVETRAIN_ENCODER_RIGHT = LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT;
        public static final String LABEL_DRIVETRAIN_ENCODER_REAR = LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK;

        public static final RevHubOrientationOnRobot.LogoFacingDirection CONTROLHUB_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection CONTROLHUB_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        public static final String LABEL_DRIVETRAIN_SENSOR_COLLISION_LEFT = "collision_sensor_left";
        public static final String LABEL_DRIVETRAIN_SENSOR_COLLISION_RIGHT = "collision_sensor_right";

        // Lighting
        public static final String LABEL_CONTROLLER_LIGHTING = "lighting_control";

        public static final boolean ENABLE_LIGHTING = Lighting.LIGHTING_ENABLED;

        // Intake
        public static final String LABEL_INTAKE_MOTOR_STAGE_ONE = "stage_one_intake";
        public static final String LABEL_INTAKE_MOTOR_STAGE_TWO = "stage_two_intake";

        public static final String LABEL_INTAKE_SERVO_SWEEPER_LEFT = "sweeper_left_intake_servo";
        public static final String LABEL_INTAKE_SERVO_SWEEPER_RIGHT = "sweeper_right_intake_servo";
        public static final String LABEL_INTAKE_SERVO_SLOT_ONE = "slot_one_servo_intake";
        public static final String LABEL_INTAKE_SERVO_SLOT_TWO = "slot_two_servo_intake";
        public static final String LABEL_INTAKE_SERVO_PIVOT = "pivot_intake";

        public static final String LABEL_INTAKE_SENSOR_SLOT_ONE = "slot_one_sensor_intake";
        public static final String LABEL_INTAKE_SENSOR_SLOT_TWO = "slot_two_sensor_intake";
        public static final String LABEL_INTAKE_SENSOR_TRACKING = "intake_sensor_tracker";

        public static final String LABEL_INTAKE_COUNT_CHECK_LOWER = "intake_count_lower_limit";
        public static final String LABEL_INTAKE_COUNT_CHECK_UPPER = "intake_count_upper_limit";
        public static final String LABEL_ARM_SENSOR_LIMIT_LOWER = "arm_sensor_lower_limit";

        // Arm
        public static final String LABEL_ARM_MOTOR_LEFT_SIDE = "left_side_arm";
        public static final String LABEL_ARM_MOTOR_RIGHT_SIDE = "right_side_arm";

        // Endgame - Drone
        public static final String LABEL_DRONE_LAUNCH_SERVO_MAIN = "drone_launch_servo";
        public static final String LABEL_DRONE_PIVOT_SERVO_MAIN = "drone_pivot_servo";

        // Vision
        public static final String LABEL_FRONT_AI_CAMERA = "front_ai_camera";

        public static final String LABEL_ALLIANCE_COLOR_SENSOR = "alliance_tag_sensor";

    }

    /**
     *  <h2>Robot Constant Values - Common</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Defines common constants used throughout.
     * Can include timeout values, math functions, etc.
     * </p>
     * <li>Command Sleep Timeout(s)</li>
     * <li>Math Constants</li>
     * <br>
     */
    public static final class CommonSettings {
        public static final int SLEEP_TIMER_MILLISECONDS_DEFAULT = 100;

        // Initialization Display
        public static final String INIT_SETTING_DISPLAY_MODE_AUTONOMOUS = "autonomous";
        public static final String INIT_SETTING_DISPLAY_MODE_TELEOP = "teleop";

        // Transition from Autonomous to Teleop - Global Static variables (not constants)
        public static double IMU_TRANSITION_ADJUSTMENT = 0;

        // Get Properties
        public static double getImuTransitionAdjustment() {
            return IMU_TRANSITION_ADJUSTMENT; }

        // Set Properties
        public static void setImuTransitionAdjustment(double inHeading) {
            IMU_TRANSITION_ADJUSTMENT = inHeading; }

    }

    /**
     *  <h2>Robot Constant Values - Drivetrain</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Drivetrain constants
     * </p>
     * <li>Drivetrain Constants</li>
     * <br>
     */
    public static final class Drivetrain {

        public static final class Autonomous {
            public static final double MOTOR_MAX_VELOCITY_FINETUNE = 15;
            public static final double MOTOR_MAX_VELOCITY_CROSSFIELD = 28;
        }

        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_DRIVE_MOTOR = "goBuilda Yellow Jacket 5203-2402-0019";
        public static final String COMMENT_DRIVE_WHEEL_TYPE = "Mecanum";

        // List Item - Drivetrain Mode Type(s)
        public static final String LIST_MODE_TYPE_DRIVETRAIN_FIELDCENTRIC = "Field Centric";
        public static final String LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC = "Robot Centric";

        // Physical Robot Settings
        public static final double DRIVETRAIN_TRACK_WIDTH_INCHES = 10; // !! need to tune !!
        public static final double WHEEL_DIAMETER_MILLIMETER = 96; // !! need to tune !!
        public static final double WHEEL_DIAMETER_INCHES = convertMillimetersToInches(WHEEL_DIAMETER_MILLIMETER);

        public static final double WHEEL_RADIUS_MILLIMETER = WHEEL_DIAMETER_MILLIMETER / 2;
        public static final double WHEEL_RADIUS_INCHES = convertMillimetersToInches(WHEEL_RADIUS_MILLIMETER);

        public static final double EXTERNAL_GEAR_RATIO = 1.0; // 1 = No External Gearing // !! need to tune !!

        // Motor/Encoder Configuration Settings
        public static final double MOTOR_MAX_RPM = 312; // Set from Vendor Specs
        public static final double ENCODER_WHEEL_DIAMETER_MILLIMETER = 48; // !! need to tune !!
        public static final double ENCODER_WHEEL_DIAMETER_INCHES = convertMillimetersToInches(ENCODER_WHEEL_DIAMETER_MILLIMETER);
        public static final double ENCODER_TICKS_PER_REV = 2000;  // 537.7; // Set from Vendor Specs

        public static final double ENCODER_TICKS_PER_INCH = (ENCODER_TICKS_PER_REV * EXTERNAL_GEAR_RATIO) /
                (ENCODER_WHEEL_DIAMETER_INCHES * Math.PI);

        // Motor Encoder PIDF Settings
        public static final double MOTOR_VELOCITY_P = 0; // !! need to tune !!
        public static final double MOTOR_VELOCITY_I = 0; // !! need to tune !!
        public static final double MOTOR_VELOCITY_D = 0; // !! need to tune !!
        public static final PIDFCoefficients DRIVE_MOTOR_VELOCITY_PIDF = new PIDFCoefficients(
                MOTOR_VELOCITY_P,
                MOTOR_VELOCITY_I,
                MOTOR_VELOCITY_D,
                getMotorVelocityF(MOTOR_MAX_RPM / 60 * ENCODER_TICKS_PER_REV));

        // Proportional control coefficient (or GAIN) for "heading control"
        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
        // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
        // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
        public static final double HEADING_P_TURN_GAIN = 0.02;   // Larger is more responsive, but also less stable
        public static final double HEADING_P_DRIVE_GAIN = 0.03;  // Larger is more responsive, but also less stable

        // These constants define the desired driving/control characteristics
        // They can/should be tweaked to suit the specific robot drive train.
        public static final double AUTO_DRIVE_SPEED = 0;         // Max driving speed for better distance accuracy.
        public static final double AUTO_TURN_SPEED = 0;          // Max Turn speed to limit turn rate
        public static final double AUTO_HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

        // Motor Encoder Feed Forward Configuration(s)
        public static final double FEED_FORWARD_KV = 1.0 / getRpmToVelocity(MOTOR_MAX_RPM); // volts * seconds / distance
        public static final double FEED_FORWARD_KA = 0; // volts * seconds^2 / distance
        public static final double FEED_FORWARD_KS = 0; // volts

        // Motor Limit 'Max' Settings
        public static final double LIMIT_MAX_VELOCITY = 30; // !! need to tune !!
        public static final double LIMIT_MAX_ACCELERATION = 30; // !! need to tune !!
        public static final double LIMIT_MAX_ANGLE_VELOCITY = Math.toRadians(60); // !! need to tune !!
        public static final double LIMIT_MAX_ANGLE_ACCELERATION = Math.toRadians(60); // !! need to tune !!


        // Motor Output Setting(s)
        public static final double MOTOR_LATERAL_MOVEMENT_STRAFING_CORRECTION = 1.1;
        public static final double MOTOR_OUTPUT_POWER_MAX = 1;

        public static final double MOTOR_OUTPUT_POWER_HIGH = 1;
        public static final double MOTOR_OUTPUT_POWER_MED = 0.75; // normal run-mode
        public static final double MOTOR_OUTPUT_POWER_LOW = 0.40; // lowered 11.27 - from .50 to .40
        public static final double MOTOR_OUTPUT_POWER_SNAIL = 0.25;

        // Sensor Settings
        public static final double SENSOR_COLLISION_LEFT_BOARD_RANGE = 3.5;
        public static final double SENSOR_COLISION_RIGHT_BOARD_RANGE = SENSOR_COLLISION_LEFT_BOARD_RANGE;
        public static final DistanceUnit LIMIT_SENSOR_DISTANCE_UNIT = DistanceUnit.INCH;


        /**
         * <h2>Convert: Encoder Ticks to Inches</h2>
         * <hr>
         * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Conversion Method: Convert Encoder Ticks/Pulses/Cycles to Inches
         * </p>
         * <br>
         *
         * @param inTicks
         *
         * @return double (Inches)
         */
        public static double getEncoderTicksToInches(double inTicks) {
            double outInches = 0;

            outInches = WHEEL_RADIUS_INCHES * 2 * Math.PI * EXTERNAL_GEAR_RATIO * inTicks / ENCODER_TICKS_PER_REV;

            return outInches;
        }

        /**
         * <h2>Convert: RPM to Velocity</h2>
         * <hr>
         * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Conversion Method: Convert RPM to Velocity
         * </p>
         * <br>
         *
         * @param inRpm
         *
         * @return double (Velocity)
         */
        public static double getRpmToVelocity(double inRpm) {
            double outVelocity = 0;

            outVelocity = inRpm * EXTERNAL_GEAR_RATIO * Math.PI * WHEEL_RADIUS_INCHES / 60.0;

            return outVelocity;
        }

        /**
         * <h2>Get Motor Velocity F Value for PIDF</h2>
         * <hr>
         * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Calculating PIDF Values from Maximum Velocity
         * Once you have your maximum velocity (maxV), you can calculate the velocity PIDF values. Your F value is calculated like this: F = 32767maxV. So if your max velocity is 2600 ticks per second (reasonable for a mechanism that uses the HD Hex Motor), then your F value should be about 12.6.
         * </p>
         * <br>
         * Source: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
         *
         * @param inTicksPerSecond
         *
         * @return double (Velocity F - for PIDF)
         */
        public static double getMotorVelocityF(double inTicksPerSecond) {
            double outVelocityF = 0;

            outVelocityF = 32767 / inTicksPerSecond;

            return outVelocityF;
        }

    }

    /**
     *  <h2>Robot Constant Values - Intake / Arm / Endgame</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Intake, Arm, and Endgame constants
     * </p>
     * <li>Motors / Servos</li>
     * <br>
     */
    public static final class IntakeArm {

        // Intake Settings
        public static final double INTAKE_MOTOR_OUTPUT_POWER_MAX = 1;
        public static final double INTAKE_MOTOR_OUTPUT_POWER_MIN = 0.5;

        // Intake Sensor Settings
        public static final DistanceUnit LIMIT_SENSOR_DISTANCE_UNIT = DistanceUnit.INCH;
        public static final double SENSOR_DISTANCE_SETPOINT_TRACKING_CLEAR = 0.80;
        public static final double SENSOR_DISTANCE_SETPOINT_TRACKING_TRIP = 0.34;
        public static final int COUNT_PIXEL_INTAKE_LIMIT = 2;

        // Servo Settings - Intake Sweeper
        public static final double SERVO_INTAKE_SWEEPER_SETPOINT_INIT = 0.0;
        public static final double SERVO_INTAKE_SWEEPER_SETPOINT_FORWARD_FULL = 1.0;
        public static final double SERVO_INTAKE_SWEEPER_SETPOINT_REVERSE_FULL = -1.0;

        // Servo Settings - Pivot
        public static final double SERVO_PIVOT_SETPOINT_HOME = 0.50;
        public static final double SERVO_PIVOT_SETPOINT_BOARD = 0.75;
//        public static final double SERVO_PIVOT_SETPOINT_AUTOPIXEL = 0.70;
//        public static final double SERVO_PIVOT_SETPOINT_AUTOPIXEL_INSIDE = 0.75;
        public static final double SERVO_PIVOT_SETPOINT_AUTOPIXEL_LVL1 = 0.70;
        public static final double SERVO_PIVOT_SETPOINT_AUTOPIXEL_LVL2 = 0.75;
        public static final double SERVO_PIVOT_SETPOINT_AUTOPIXEL_LVL3 = 0.75;

        // Servo Settings - Pixel Slot
        public static final double SERVO_SLOTONE_SETPOINT_INIT = 0.40;
        public static final double SERVO_SLOTONE_SETPOINT_OPEN = 1;
        public static final double SERVO_SLOTONE_SETPOINT_CLOSE = 0.40;

        public static final double SERVO_SLOTTWO_SETPOINT_INIT = 0.64;
        public static final double SERVO_SLOTTWO_SETPOINT_OPEN = 1;
        public static final double SERVO_SLOTTWO_SETPOINT_CLOSE = 0.64;

        // Servo Settings - Endgame - Drone
        public static final double SERVO_DRONE_LAUNCH_SETPOINT_INIT = 0.50;
        public static final double SERVO_DRONE_LAUNCH_SETPOINT_OPEN = 0;
        public static final double SERVO_DRONE_LAUNCH_SETPOINT_CLOSE = 0.50;

        public static final double SERVO_DRONE_PIVOT_SETPOINT_INIT = 0.50;
        public static final double SERVO_DRONE_PIVOT_SETPOINT_LAUNCH = 0.25;

        // Arm Settings
        public static final double ARM_MOTOR_OUTPUT_POWER_MAX = 1;
        public static final double ARM_MOTOR_OUTPUT_POWER_MIN = 0.5;

        // Arm - Setpoints
        public static final int ARM_ENCODER_SETPOINT_HOME = 0;
        public static final int ARM_ENCODER_SETPOINT_MAX = 4300; // was 8200 (replaced motor)
        public static final int ARM_ENCODER_SETPOINT_CRUISE = 800;
        public static final int ARM_ENCODER_SETPOINT_AUTOPIXEL_LVL1 = 1850; // was 2050
        public static final int ARM_ENCODER_SETPOINT_AUTOPIXEL_LVL2 = 2050; // was 2050
        public static final int ARM_ENCODER_SETPOINT_AUTOPIXEL_LVL3 = 2250; // was 2050
        public static final int ARM_ENCODER_SETPOINT_LOW = 1950; // was 2050
        public static final int ARM_ENCODER_SETPOINT_PRECLIMB = 2900; // was 5000 (replaced motor) // was 2600 // added 300 for drone launcher
        public static final int ARM_ENCODER_SETPOINT_HANG = 680; // was 1300 (replaced motor)

        public static final int ARM_ENCODER_LIMIT_CHECK_INTAKE = 50;
    }

    /**
     *  <h2>Robot Constant Values - Vision and Sensors</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Vision and Sensor constants
     * </p>
     * <li>Vision</li>
     * <li>Sensors</li>
     * <br>
     */
    public static final class Vision {

        public static final class AI_Camera {

            // AI Camera Settings

            // AI Camera Mode(s)
            public static final String AI_CAMERA_MODE_APRILTAG = "april_tag";
            public static final String AI_CAMERA_MODE_OBJECT_TRACKING = "object_tracking";
            public static final String AI_CAMERA_MODE_OBJECT_RECOGNITION = "object_recognition";

        }

        public static final class Sensor {

            // Sensor - Alliance Sensor
            public static final boolean ALLIANCE_SENSOR_LED_ACTIVE = true;

        }

        // AI Camera Setpoints - Target Zone(s)
        public static final int RANDOM_TARGET_ZONE_ONE_X = 150;
        public static final int RANDOM_TARGET_ZONE_TWO_X = 300;
        public static final int RANDOM_TARGET_ZONE_THREE_X = 320;



    }

    /**
     *  <h2>Robot Constant Values - Lighting</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Lighting constants
     * </p>
     * <li>Lighting Constants</li>
     *
     * CP1 = Green
     * CP2 = Blue
     *
     * CP1_2_COLOR_GRADIENT
     * BEATS_PER_MINUTE_FOREST_PALETTE
     * BEATS_PER_MINUTE_OCEAN_PALETTE
     * BEATS_PER_MINUTE_PARTY_PALETTE
     * SHOT_WHITE
     * CP2_SHOT
     * CP2_BREATH_SLOW
     * CP2_HEARTBEAT_SLOW
     * CP2_LARSON_SCANNER
     *
     * CP2_LIGHT_CHASE
     *
     * CONFETTI
     * RAINBOW_LAVA_PALETTE
     * RAINBOW_PARTY_PALETTE
     * RAINBOW_WITH_GLITTER
     * RAINBOW_FOREST_PALETTE
     * RAINBOW_OCEAN_PALETTE
     *
     * CP1_2_SPARKLE_2_ON_1
     * CP1_2_SINELON
     * CP1_2_BEATS_PER_MINUTE
     *
     * CP1_2_TWINKLES **
     * TWINKLES_FOREST_PALETTE **
     *
     * CP1_2_NO_BLENDING
     *
     * CP1_STROBE
     *
     * STROBE_BLUE
     *
     * FIRE_LARGE
     *
     *
     *
     * HEARTBEAT_BLUE
     * <br>
     */
    public static final class Lighting {

        // Lighting Enabled (True/False)
        // -- Only enabled if a lighting controller and lighting is installed on the robot
        public static final boolean LIGHTING_ENABLED = true;

        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_LIGHTING_CONTROLLER = "REV-11-1105 Blinkin LED Driver";

        // Patterns to Avoid
        public static final String LIGHT_PATTERN_AVOID_KEYWORD_TWINKLES = "Twinkles";

        // Default Light Pattern
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_DEFAULT = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_DEFAULT_TELEOP = LIGHT_PATTERN_DEFAULT;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_DEFAULT_AUTONOMOUS = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_PREGAME_OPTION_CONFIG = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;

        // System Initialize Light Patter/state
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_LIGHTING = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST; // CP1_STROBE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN = RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST; // STROBE_WHITE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_VISION = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW; // CP2_STROBE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_INTAKEARM = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE; // STROBE_GOLD
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_CLAW = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED; // STROBE_RED
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_TARGETING = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST; // STROBE_RED
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_SOUND = RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE; // STROBE_RED
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_COMPLETE = RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER; // RAINBOW_FOREST_PALETTE;

        // Light Pattern(s) for Autonomous
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE = RevBlinkinLedDriver.BlinkinPattern.GOLD;    // Peach
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO = RevBlinkinLedDriver.BlinkinPattern.AQUA;    // Trojan
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE = RevBlinkinLedDriver.BlinkinPattern.GREEN; // Cards
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID = RevBlinkinLedDriver.BlinkinPattern.RED;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_ONE = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER; // RAINBOW_WITH_GLITTER
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_TWO = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE; // RAINBOW_PARTY_PALETTE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_THREE = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE; // RAINBOW_RAINBOW_PALETTE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_INVALID = LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID;

        // Intake
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_INTAKE_CAPACITY_LIMIT_UPPER = RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_INTAKE_CAPACITY_LIMIT_LOWER = RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_INTAKE_UNAVAILABLE = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;

        // Light Patterns for Robot State(s) - Linear Slide
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_MED = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_LOW = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_GROUND = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_LIMIT_MAX = RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER;

        // Light Patterns for Robot State(s) - Claw
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_CLAW_CLAMP_OPEN = RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE; // LIGHT_CHASE_RED
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_CLAW_CLAMP_CLOSED = RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE; // LIGHT_CHASE_BLUE

        // Light Patterns for Targeting
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_TARGET_JUNCTION_ZONE_ACTIVE = RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_ALERT_ENDGAME = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;

    }

    // --------------------------------------------
    // General Method(s)
    // --------------------------------------------


    /**
     * <h2>utilRobotConstants Method: convertMillimetersToInches</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Convert a value from millimeters to inches
     * </p>
     * @param inValueInMillimeters - value to be converted (in millimeters)
     *
     * @return double - value converted to inches
     */
    public static double convertMillimetersToInches(double inValueInMillimeters) {
        return inValueInMillimeters / CONVERSION_FACTOR_MM_INCH;
    }

    /**
     * <h2>RobotConstants Method: convertInchesToMillimeters</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Convert a value from inches to millimeters
     * </p>
     * @param inValueInInches - value to be converted (in inches)
     *
     * @return double - value converted to millimeters
     */
    public static double convertInchesToMillimeters(double inValueInInches) {
        return inValueInInches * CONVERSION_FACTOR_MM_INCH;
    }

    public static double convertRadiansToDegrees(double inValueInRadians) {
        return inValueInRadians * CONVERSION_FACTOR_RADIAN_DEGREE;
    }

    public static double convertDegreesToRadians(double inValueInDegrees) {
        return inValueInDegrees / CONVERSION_FACTOR_RADIAN_DEGREE;
    }
}
