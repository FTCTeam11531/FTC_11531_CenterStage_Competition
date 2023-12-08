package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.system.sysIntakeArm;
import org.firstinspires.ftc.teamcode.system.sysLighting;
import org.firstinspires.ftc.teamcode.system.sysDrivetrainMecanum;
import org.firstinspires.ftc.teamcode.system.sysVision;
import org.firstinspires.ftc.teamcode.utility.enumStateDriveMotorMaxOutputPower;
import org.firstinspires.ftc.teamcode.utility.enumStateDrivetrainMode;
import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

/**
 * <h2>OpMode - Teleop - Main</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Teleop Mode - Main
 * One OpMode to Rule them All
 * </p>
 * <br>
 */
@TeleOp(name="Teleop Main", group="_main")
//@Disabled
public class opmodeTeleopMain extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initialization

    // -- Lighting System
    sysLighting sysLighting = new sysLighting(this);

    // -- Drivetrain System
    sysDrivetrainMecanum sysDrivetrain = new sysDrivetrainMecanum(this);

    // Vision System
    sysVision sysVision = new sysVision(this);

    // Intake / Arm
    sysIntakeArm sysIntakeArm = new sysIntakeArm(this);

    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Set telemetry mode to append
//        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // ------------------------------------------------------------
        // Initialize System(s) - set different light mode between each system init
        // ------------------------------------------------------------

        // Initialize System(s)
        sysLighting.init(utilRobotConstants.Configuration.ENABLE_LIGHTING);
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LIGHTING);

        sysDrivetrain.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN);

        sysVision.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

        sysIntakeArm.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_INTAKEARM);

        // ------------------------------------------------------------
        // Configure drivetrain for Teleop Mode
        // ------------------------------------------------------------
        sysDrivetrain.setDriveMotorRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // ------------------------------------------------------------
        // Configure Vision
        // ------------------------------------------------------------
        // Set AI Camera Mode
//        sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_APRILTAG);
        sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_OBJECT_TRACKING);

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        double inputAxial, inputLateral, inputYaw, prevPixelTrackerMeasure;
        boolean isManualSlideMode = false;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // Reset runtime clock
        runtime.reset();

        // Set Intake Servos - deviated from initialization
        sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_CLOSE);
        sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);

        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_COMPLETE);

        // Repeat while in initialize state (disable if using waitForStart)
//        while(opModeInInit() && !isStopRequested()) {
//
//        }

        // Wait for Start state (disable if using opModeInInit)
        waitForStart();

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to auto-clear
//        telemetry.setAutoClear(true);
        telemetry.clearAll();

        // Starting LED mode
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_DEFAULT_TELEOP);

        // Reset runtime clock
        runtime.reset();

        // Return if a Stop Action is requested
        if (isStopRequested()) {

            // Update the Transition Adjustment Value for the IMU
            utilRobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

            return;
        }

        // ------------------------------------------------------------
        // Command Loop: run until the end of the match (driver presses STOP)
        // ------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {

            // ------------------------------------------------------------
            // Controls
            // ------------------------------------------------------------
            // Gamepad1 = Main Driver
            // ------------------------------------------------------------
            // -- Robot Movement
            // -- -- Axis (left_stick_x, left_stick_y): Drive
            // -- -- Axis (right_stick_x): Rotate
            //
            // -- Intake
            // -- -- Left Bumper (hold): Intake Normal - into robot
            // -- -- Right Bumper (hold): Drivetrain Speed - Low
            //
            // -- Endgame
            // -- -- Back (hold) [co-driver needed]: Drone Launch
            // -- -- Start (hold) [co-driver needed]: Drone Reset
            //
            // -- Override Settings
            // -- -- D-Pad Up + X: Reset Heading Override (and Raw)
            //
            // ------------------------------------------------------------
            // Gamepad2 = Co-Driver
            // ------------------------------------------------------------
            // -- Arm Movement
            // -- -- Axis (right_stick_x): Up/Down
            // -- -- Y: Arm Setpoint - High
            // -- -- A: Arm Setpoint - Home
            // -- -- B: Arm Setpoint - Preclimb
            // -- -- X: Arm Setpoint - Hang

            // -- Endgame
            // -- -- Back (hold) [main-driver needed]: Drone Launch
            // -- -- Start (hold) [main-driver needed]: Drone Reset
            //
            // -- Sleeve
            // -- -- D-Pad Up: Sleeve Pivot - Board
            // -- -- D-Pad Down: Sleeve Pivot - Home
            // -- -- D-Pad Left: Sleeve Release
            // -- -- D-Pad Right: Sleeve Lock
            //
            // ------------------------------------------------------------
            // Drivetrain
            // ------------------------------------------------------------
            // Assign gamepad control to motion in relation to:
            // -- gamepad input, direction
            // -- robot orientation to field
            // -- installed direction of control hub
            // -- orientation of drivetrain/motors
            inputYaw =  (gamepad1.right_stick_x);
            inputAxial = -(gamepad1.left_stick_y);
            inputLateral = (gamepad1.left_stick_x);

            // Set Field Centric as the only Drivetrain
            sysDrivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());

//            // Drivetrain Type determined by 'Drivetrain Mode' enumeration selection (Default to Field Centric)
//            if(sysDrivetrain.getLabelDrivetrainMode().equals(utilRobotConstants.Drivetrain.LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC)) {
//                // Send gamepad input for drivetrain to driveMecanum method in the drivetrain system class
//                sysDrivetrain.driveMecanum(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());
//            }
//            else {
//                // Send gamepad input for drivetrain to driveMecanumFieldCentric method in the drivetrain system class
//                sysDrivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());
//            }

            // Button Action - Set Output Power Mode to High
            if(gamepad1.right_bumper) {
                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Low;
            }

            // Button Action - Set Output Power Mode to Medium
            if(!gamepad1.right_bumper) {
                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Medium;
            }

            // ------------------------------------------------------------
            // Intake / Arm
            // ------------------------------------------------------------
            if(!isManualSlideMode) {
                if (sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE) <= utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HANG) {
                    sysIntakeArm.setArmMotorPower(utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                } else {
                    sysIntakeArm.setArmMotorPower(utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
                }
            }

            // Intake - Activate
            if(gamepad1.left_bumper) {

                if(sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE) <= utilRobotConstants.IntakeArm.ARM_ENCODER_LIMIT_CHECK_INTAKE) {
                    sysIntakeArm.activateIntake(utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_DEFAULT_TELEOP);
                }
                else {
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_INTAKE_UNAVAILABLE);
                }

                if(sysIntakeArm.counterIntakeUpper >= utilRobotConstants.IntakeArm.COUNT_PIXEL_INTAKE_LIMIT) {
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_INTAKE_CAPACITY_LIMIT_UPPER);
                }
                else if(sysIntakeArm.counterIntakeLower >= utilRobotConstants.IntakeArm.COUNT_PIXEL_INTAKE_LIMIT) {
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_INTAKE_CAPACITY_LIMIT_LOWER);
                }
                else {
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_DEFAULT_TELEOP);
                }

            }

            // Intake - Reverse
            if(gamepad1.a) {
                sysIntakeArm.reverseIntake();
            }

            // Intake - Deactivate when not pressing buttons
            if(!gamepad1.left_bumper && !gamepad1.a) {
                sysIntakeArm.deactivateIntake();
            }

            // Intake Pivot - Home
//            if(gamepad2.dpad_down) {
//                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME);
//            }

            // Intake Pivot - Board
//            if(gamepad2.dpad_up) {
//                if(sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE) >= utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HANG) {
//                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD);
//                }
//            }

            // Intake Slot - Open
            if(gamepad2.left_bumper) {
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                sleep(1500);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);

                sysIntakeArm.resetPixelTracking();
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_DEFAULT_TELEOP);
            }

            // Intake Slot - Close
//            if(gamepad2.dpad_right) {
//                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_CLOSE);
//            }

            // Reset Ground when hit limit
            if(sysIntakeArm.getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_ARM_SENSOR_LIMIT_LOWER)) {
                sysIntakeArm.resetArmEncoder();
            }

            // Arm - Manual Control
            if(Math.abs(gamepad2.right_stick_y) >= 0.25) {
                isManualSlideMode = true;
                if(sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE) >= 0 && sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE) <= utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_MAX) {
                    if(sysIntakeArm.getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_ARM_SENSOR_LIMIT_LOWER)) {
                        sysIntakeArm.moveArmManually(-(Math.abs(gamepad2.right_stick_y)), utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                    }
                    else {
                        sysIntakeArm.moveArmManually(-(gamepad2.right_stick_y), utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                    }
                }
            }
            else {
                if(isManualSlideMode) {
                    sysIntakeArm.moveArmManually(0, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                }
            }

            // Arm Setpoint - High
            if(gamepad2.y) {
                isManualSlideMode = false;
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_CLOSE);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD);
                sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_MAX, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
            }

            // Arm Setpoint - Hang
            if(gamepad2.x) {
                isManualSlideMode = false;
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_CLOSE);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD);
                sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HANG, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
            }

            // Arm Setpoint - Pre-Climb
            if(gamepad2.b) {
                isManualSlideMode = false;
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_CLOSE);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD);
                sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_PRECLIMB, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
            }

            // Arm Setpoint - Home
            if(gamepad2.a) {
                isManualSlideMode = false;
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_CLOSE);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME);
                sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
            }

            // Manual Control of pixel sleeve - to board
            if(gamepad2.dpad_up) {

                if(sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) <= utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD) {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) + 0.05);
                }

            }
            if(gamepad2.left_stick_y >= 0.5) {

                if(sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) <= utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD) {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) + 0.05);
                }

            }

            // Manual Control of pixel sleeve - to home
            if(gamepad2.dpad_down) {

                if(sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) >= utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME) {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) - 0.05);
                }

            }
            if(gamepad2.left_stick_y <= -0.5) {

                if(sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) >= utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME) {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) - 0.05);
                }

            }

            // ------------------------------------------------------------
            // Vision
            // ------------------------------------------------------------


            // ------------------------------------------------------------
            // Sensor
            // ------------------------------------------------------------
            sysIntakeArm.checkSensorPixelTracking();

            // ------------------------------------------------------------
            // Override
            // ------------------------------------------------------------
            // Button Action - Reset Heading Override (and Raw)
            if(gamepad1.start && gamepad1.y) {

                // Reset the Robot Heading (normally done on init of Drivetrain system)
                sysDrivetrain.resetZeroRobotHeading();

                // Cycle Pause
//                sleep(utilRobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
            }

            // ------------------------------------------------------------
            // Endgame
            // ------------------------------------------------------------
            if(runtime.time() >= 90.00 && runtime.time() <= 120.00) {
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_ALERT_ENDGAME);
            }

            if (gamepad1.start && gamepad2.start) {
                double currentRuntime = getRuntime();

//                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_DRONE_PIVOT_SERVO_MAIN, utilRobotConstants.IntakeArm.SERVO_DRONE_PIVOT_SETPOINT_LAUNCH);
//                sleep(500);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_DRONE_LAUNCH_SERVO_MAIN, utilRobotConstants.IntakeArm.SERVO_DRONE_LAUNCH_SETPOINT_OPEN);
            }

            if (gamepad1.back && gamepad2.back) {
                double currentRuntime = getRuntime();

//                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_DRONE_PIVOT_SERVO_MAIN, utilRobotConstants.IntakeArm.SERVO_DRONE_PIVOT_SETPOINT_INIT);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_DRONE_LAUNCH_SERVO_MAIN, utilRobotConstants.IntakeArm.SERVO_DRONE_LAUNCH_SETPOINT_INIT);
            }


//            // ------------------------------------------------------------
//            // Driver Hub Feedback
//            // ------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
//
//            // ------------------------------------------------------------
//            // - Gamepad telemetry
//            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Gamepad");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Gamepad 1 - [Y] Axial", "%4.2f", gamepad1.left_stick_y);
//            telemetry.addData("Gamepad 1 - [X] Lateral", "%4.2f", gamepad1.left_stick_x);
//            telemetry.addData("Gamepad 1 - [R] Rotation", "%4.2f", gamepad1.right_stick_x);
//
//            // ------------------------------------------------------------
//            // - Drivetrain telemetry
//            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Drivetrain");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Drivetrain Mode", sysDrivetrain.getLabelDrivetrainMode());
//            telemetry.addData("Drivetrain Power", sysDrivetrain.getLabelDrivetrainOutputPower());
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Power Front left/Right", "%4.2f, %4.2f"
//                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
//                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
//            telemetry.addData("Power Back  left/Right", "%4.2f, %4.2f"
//                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
//                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Encoder Front left/Right", "%7d, %7d"
//                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
//                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
//            telemetry.addData("Encoder Back  left/Right", "%7d, %7d"
//                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
//                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Robot Heading Raw", sysDrivetrain.getRobotHeadingRaw());
//            telemetry.addData("Heading Adjustment", utilRobotConstants.CommonSettings.getImuTransitionAdjustment());
//            telemetry.addData("Robot Heading (Adjusted)", sysDrivetrain.getRobotHeadingAdj());
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Robot Angle - Yaw (Z)", sysDrivetrain.getRobotAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Robot Angle - Pitch (X)", sysDrivetrain.getRobotAngles().getPitch(AngleUnit.DEGREES));
//            telemetry.addData("Robot Angle - Yaw (Z)", sysDrivetrain.getRobotAngles().getRoll(AngleUnit.DEGREES));
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Robot Angle Velocity - Yaw (Z)", sysDrivetrain.getRobotAngularVelocity().zRotationRate);
//            telemetry.addData("Robot Angle Velocity - Pitch (X)", sysDrivetrain.getRobotAngularVelocity().xRotationRate);
//            telemetry.addData("Robot Angle Velocity - Yaw (Z)", sysDrivetrain.getRobotAngularVelocity().yRotationRate);
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Collision Detection - Left", sysDrivetrain.getSensorDistanceCollision(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_SENSOR_COLLISION_LEFT));
//            telemetry.addData("Collision Detection - Right", sysDrivetrain.getSensorDistanceCollision(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_SENSOR_COLLISION_RIGHT));
//
//            // ------------------------------------------------------------
//            // - Vision telemetry
//            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Vision");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Camera Block Count", sysVision.getCameraObjectList().length);
//            telemetry.addData("Alliance Color", sysVision.getDetectedAllianceTagColor());
////            telemetry.addData("R-G-B", "%4, %4, %4"
////                    , sysVision.getAllianceTagColorLevel("red")
////                    , sysVision.getAllianceTagColorLevel("green")
////                    , sysVision.getAllianceTagColorLevel("blue"));
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Sensor");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Intake Count", sysIntakeArm.counterPixelCount);
//            telemetry.addData("Intake - Lower - Count", sysIntakeArm.counterIntakeLower);
//            telemetry.addData("Intake - Upper - Count", sysIntakeArm.counterIntakeUpper);
////            telemetry.addData("Pixel Tracking Sensor Distance", sysIntakeArm.getSensorDistance(utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_TRACKING));
//
//            // ------------------------------------------------------------
//            // - Intake / Arm telemetry
//            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Intake / Arm");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Pivot Position", sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT));
//            telemetry.addData("Slot One Position", sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE));
//            telemetry.addData("Slot Two Position", sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO));
//            telemetry.addData("Arm Position - Left", sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE));
//            telemetry.addData("Arm Position - Right", sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_RIGHT_SIDE));
//            telemetry.addData("Arm Limit - Lower", sysIntakeArm.getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_ARM_SENSOR_LIMIT_LOWER));
//            telemetry.addData("Intake Counter - Lower", sysIntakeArm.getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_LOWER));
//            telemetry.addData("Intake Counter - Upper", sysIntakeArm.getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_UPPER));
//
//            // ------------------------------------------------------------
//            // - Lighting telemetry
//            // ------------------------------------------------------------
//            if(utilRobotConstants.Configuration.ENABLE_LIGHTING) {
//                telemetry.addData("-", "------------------------------");
//                telemetry.addData("-", "-- Lighting");
//                telemetry.addData("-", "------------------------------");
//                telemetry.addData("Pattern", sysLighting.ledLightPattern.toString());
//            }

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "(reset robot)");
            telemetry.addData("-", "-- If you see an I2C error");
            telemetry.addData("-", "");
            telemetry.addData("-", "(change battery)");
            telemetry.addData("-", "-- if expansion hub loses connection");
            telemetry.addData("-", "-- if voltage below 12v");
            telemetry.addData("-", "");
            telemetry.addData("-", "Take your time!");
            telemetry.addData("-", "");
            telemetry.addData("-", "Drive with care and caution");
            telemetry.addData("-", "");
            telemetry.addData("-", "Don't drive like you stole it");
            telemetry.update();

//            // Input assignment to 'pause' telemetry update(s)
//            if (!gamepad1.dpad_right) {
//            }

            // Pace this loop so commands move at a reasonable speed.
//            sleep(utilRobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
        }

        // ------------------------------------------------------------
        // Closing Teleop
        // ------------------------------------------------------------
        // Update the Transition Adjustment Value for the IMU
        utilRobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

    }
}
