package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

import java.util.Arrays;
import java.util.List;

/**
 * <h2>System - Intake and Arm</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * System that contains attributes and methods for:
 * intake and armature control systems.
 * Endgame controls are also included in this system (drone launch)
 * </p>
 * <br>
 */
public class sysIntakeArm {

    private LinearOpMode sysOpMode = null;

    private DcMotorEx stageOneIntake, stageTwoIntake, leftSideArm, rightSideArm;
    private List<DcMotorEx> listMotorsIntake, listMotorsArm;

    private Servo slotOneIntakeServo, slotTwoIntakeServo, pivotIntakeServo, droneLaunchServo, dronePivotServo;
    private CRServo sweeperLeftIntakeServo, sweeperRightIntakeServo;
    private DistanceSensor limitSlotOneSensor, limitSlotTwoSensor, trackIntakeSensor;
    private TouchSensor limitArmLowerSensor, limitIntakeCountCheckLower, limitIntakeCountCheckUpper;

    public int counterPixelCount = 0, counterIntakeLower = 0, counterIntakeUpper = 0;
    private boolean isPixelFoundCounter = false, isPixelFoundIntakeLower = false, isPixelFoundIntakeUpper = false;

    public sysIntakeArm(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    public void init() {

        // Intake
        stageOneIntake = sysOpMode.hardwareMap.get(DcMotorEx.class, utilRobotConstants.Configuration.LABEL_INTAKE_MOTOR_STAGE_ONE);
        stageTwoIntake = sysOpMode.hardwareMap.get(DcMotorEx.class, utilRobotConstants.Configuration.LABEL_INTAKE_MOTOR_STAGE_TWO);

        // Add Intake motors to array
        listMotorsIntake = Arrays.asList(stageOneIntake, stageTwoIntake);

        // Intake Servo
        sweeperLeftIntakeServo = sysOpMode.hardwareMap.get(CRServo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_LEFT);
        sweeperRightIntakeServo = sysOpMode.hardwareMap.get(CRServo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_RIGHT);
        slotOneIntakeServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE);
        slotTwoIntakeServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO);
        pivotIntakeServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT);

        droneLaunchServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_DRONE_LAUNCH_SERVO_MAIN);
        dronePivotServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_DRONE_PIVOT_SERVO_MAIN);

//        limitSlotOneSensor = sysOpMode.hardwareMap.get(DistanceSensor.class, utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_SLOT_ONE);
//        limitSlotTwoSensor = sysOpMode.hardwareMap.get(DistanceSensor.class, utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_SLOT_TWO);
//        trackIntakeSensor = sysOpMode.hardwareMap.get(DistanceSensor.class, utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_TRACKING);
        limitArmLowerSensor = sysOpMode.hardwareMap.get(TouchSensor.class, utilRobotConstants.Configuration.LABEL_ARM_SENSOR_LIMIT_LOWER);
        limitIntakeCountCheckLower = sysOpMode.hardwareMap.get(TouchSensor.class, utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_LOWER);
        limitIntakeCountCheckUpper = sysOpMode.hardwareMap.get(TouchSensor.class, utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_UPPER);

        // Arm
        leftSideArm = sysOpMode.hardwareMap.get(DcMotorEx.class, utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE);
        rightSideArm = sysOpMode.hardwareMap.get(DcMotorEx.class, utilRobotConstants.Configuration.LABEL_ARM_MOTOR_RIGHT_SIDE);

        // Add linear arm motors to array
        listMotorsArm = Arrays.asList(leftSideArm, rightSideArm);

        // Configuration / Initialize Hardware
        stageOneIntake.setDirection(DcMotorEx.Direction.REVERSE);
        stageTwoIntake.setDirection(DcMotorEx.Direction.REVERSE);

        leftSideArm.setDirection(DcMotorEx.Direction.FORWARD);
        rightSideArm.setDirection(DcMotorEx.Direction.REVERSE);

//        setIntakeMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setArmMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        resetArmEncoder();

        // Servo Initialization Point(s)
        sweeperLeftIntakeServo.setPower(utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_INIT);
        sweeperRightIntakeServo.setPower(utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_INIT);
        pivotIntakeServo.setPosition(utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME);
        slotOneIntakeServo.setPosition(utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_INIT);
        slotTwoIntakeServo.setPosition(utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_INIT);

        droneLaunchServo.setPosition(utilRobotConstants.IntakeArm.SERVO_DRONE_LAUNCH_SETPOINT_INIT);
        dronePivotServo.setPosition(utilRobotConstants.IntakeArm.SERVO_DRONE_PIVOT_SETPOINT_INIT);

        // Display telemetry
        sysOpMode.telemetry.addData(">", "------------------------------------");
        sysOpMode.telemetry.addData(">", " System: Intake/Arm Initialized");
        sysOpMode.telemetry.update();

    }

    public void activateIntake(double inIntakePower) {

        // Reverse Stage One
        if(counterIntakeLower < utilRobotConstants.IntakeArm.COUNT_PIXEL_INTAKE_LIMIT) {
            // Sweeper - Spin forward - into intake
            setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_LEFT, utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_FORWARD_FULL);
            setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_RIGHT, utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_REVERSE_FULL);
        }
        else {
            // Activate Slot Two Servo
//            setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_CLOSE);

            //Sweeper - Spin backward - keep out
            setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_LEFT, utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_REVERSE_FULL);
            setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_RIGHT, utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_FORWARD_FULL);

        }

        // Reverse Stage Two
        if(counterIntakeUpper < utilRobotConstants.IntakeArm.COUNT_PIXEL_INTAKE_LIMIT) {

            // Power Intake Motors - Forward
            setIntakeMotorPower(inIntakePower);
        }
        else {

            // Power Intake Motors - Reverse
            setIntakeMotorPower(-inIntakePower);

        }
    }

    public void deactivateIntake() {

        // Stop Intake Motors
        setIntakeMotorPower(0);
        setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_LEFT, utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_INIT);
        setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_RIGHT, utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_INIT);
    }

    public void reverseIntake() {

        // Power Intake Motors - Reverse
        setIntakeMotorPower(-(utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX));

        //Sweeper - Spin backward - keep out
        setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_LEFT, utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_REVERSE_FULL);
        setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_RIGHT, utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_FORWARD_FULL);
    }

    public void checkSensorPixelTracking() {

        if(getSensorDistance(utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_TRACKING) > utilRobotConstants.IntakeArm.SENSOR_DISTANCE_SETPOINT_TRACKING_CLEAR && isPixelFoundCounter) {
            counterPixelCount ++;
            isPixelFoundCounter = false;
        }
        else if (getSensorDistance(utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_TRACKING) < utilRobotConstants.IntakeArm.SENSOR_DISTANCE_SETPOINT_TRACKING_CLEAR) {
            isPixelFoundCounter = true;
        }

        if(!getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_LOWER) && isPixelFoundIntakeLower) {
            counterIntakeLower ++;
            isPixelFoundIntakeLower = false;
        }
        else if (getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_LOWER)) {
            isPixelFoundIntakeLower = true;
        }

        if(!getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_UPPER) && isPixelFoundIntakeUpper) {
            counterIntakeUpper ++;
            isPixelFoundIntakeUpper = false;
        }
        else if (getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_UPPER)) {
            isPixelFoundIntakeUpper = true;
        }

    }

    public void resetPixelTracking() {

        counterPixelCount = 0;
        counterIntakeLower = 0;
        counterIntakeUpper = 0;
    }

    public void resetArmEncoder() {

        // Reset encoder
        setArmMotorRunMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setArmMotorPower(0);
    }

    public void moveArmToTarget(int inTargetSetPoint, double inMaxOutputPowerPercent) {

        // Configure Motor Target Set Point and Set motor as Run to Position
        setArmTargetPosition(inTargetSetPoint);
        setArmMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Move linear slide within min/max limits
        setArmMotorPower(inMaxOutputPowerPercent);

    }

    public void moveArmManually(double inAppliedPower, double inMaxOutputPowerPercent) {

        // Configure Motor Target Set Point and Set motor as Run to Position
        setArmMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Move linear slide within min/max limits
        setArmMotorPower((inAppliedPower * inMaxOutputPowerPercent));
    }

    public double getIntakeServoPosition(String inIntakeServoName) {
        double outPosition;

        switch(inIntakeServoName) {
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE):
                outPosition = slotOneIntakeServo.getPosition();
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO):
                outPosition = slotTwoIntakeServo.getPosition();
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT):
                outPosition = pivotIntakeServo.getPosition();
                break;
            case(utilRobotConstants.Configuration.LABEL_DRONE_LAUNCH_SERVO_MAIN):
                outPosition = droneLaunchServo.getPosition();
                break;
            case(utilRobotConstants.Configuration.LABEL_DRONE_PIVOT_SERVO_MAIN):
                outPosition = dronePivotServo.getPosition();
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_LEFT):
                outPosition = sweeperLeftIntakeServo.getPower();
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_RIGHT):
                outPosition = sweeperRightIntakeServo.getPower();
                break;
            default:
                outPosition = 0;
        }

        return outPosition;
    }

    public double getSensorDistance(String inRangeSensorName) {
        double outDistance;

        switch(inRangeSensorName) {
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_SLOT_ONE):
                outDistance = limitSlotOneSensor.getDistance(utilRobotConstants.IntakeArm.LIMIT_SENSOR_DISTANCE_UNIT);
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_SLOT_TWO):
                outDistance = limitSlotTwoSensor.getDistance(utilRobotConstants.IntakeArm.LIMIT_SENSOR_DISTANCE_UNIT);
                break;
//            case(utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_TRACKING):
//                outDistance = trackIntakeSensor.getDistance(utilRobotConstants.IntakeArm.LIMIT_SENSOR_DISTANCE_UNIT);
//                break;
            default:
                outDistance = 0;
        }

        return outDistance;
    }

    public boolean getLimitSensorTripped(String inSensorLabel) {
        boolean outLimitSensorTripped = false;

        // Get boolean value from Touch Sensor
        switch (inSensorLabel) {
            // Arm Limit Ground Sensor
            case utilRobotConstants.Configuration.LABEL_ARM_SENSOR_LIMIT_LOWER:
                outLimitSensorTripped = limitArmLowerSensor.isPressed();
                break;
            // Intake Lower Check
            case utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_LOWER:
                outLimitSensorTripped = limitIntakeCountCheckLower.isPressed();
                break;
            // Intake Upper Check
            case utilRobotConstants.Configuration.LABEL_INTAKE_COUNT_CHECK_UPPER:
                outLimitSensorTripped = limitIntakeCountCheckUpper.isPressed();
                break;
            // Default - No match
            default:
                outLimitSensorTripped = false;
        }

        return outLimitSensorTripped;
    }

    public int getArmCurrentPosition(String inMotorLabel) {
        // Variable for output Power value for drivetrain motor(s)
        int outEncoderPosition;

        // Get value for motor specified in method call
        switch (inMotorLabel) {
            // Linear Slide Motor - Left
            case utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE:
                outEncoderPosition = leftSideArm.getCurrentPosition();
                break;
            // Linear Slide Motor - Right
            case utilRobotConstants.Configuration.LABEL_ARM_MOTOR_RIGHT_SIDE:
                outEncoderPosition = rightSideArm.getCurrentPosition();
                break;
            // Default - No match
            default:
                outEncoderPosition = 0;
        }

        // Return value
        return outEncoderPosition;
    }

    public void setIntakeServoPosition(String inIntakeServoName, double inTargetPosition) {

        switch(inIntakeServoName) {
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE):
                slotOneIntakeServo.setPosition(inTargetPosition);
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO):
                slotTwoIntakeServo.setPosition(inTargetPosition);
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT):
                pivotIntakeServo.setPosition(inTargetPosition);
                break;
            case(utilRobotConstants.Configuration.LABEL_DRONE_LAUNCH_SERVO_MAIN):
                droneLaunchServo.setPosition(inTargetPosition);
                break;
            case(utilRobotConstants.Configuration.LABEL_DRONE_PIVOT_SERVO_MAIN):
                dronePivotServo.setPosition(inTargetPosition);
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_LEFT):
                // Set Continuous Servo direction
                if(inTargetPosition < 0) {
                    sweeperLeftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                else {
                    sweeperLeftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                sweeperLeftIntakeServo.setPower(Math.abs(inTargetPosition));
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER_RIGHT):
                // Set Continuous Servo direction
                if(inTargetPosition < 0) {
                    sweeperRightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                else {
                    sweeperRightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                sweeperRightIntakeServo.setPower(Math.abs(inTargetPosition));
                break;
        }

    }

    public void setArmTargetPosition(int inTargetPosition) {
        for (DcMotorEx itemMotor: listMotorsArm) {
            itemMotor.setTargetPosition(inTargetPosition);
        }
    }

    public void setArmMotorRunMode(DcMotorEx.RunMode inRunMode) {
        for (DcMotorEx itemMotor: listMotorsArm) {
            itemMotor.setMode(inRunMode);
        }
    }

    public void setArmMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior inZeroPowerBehavior) {
        for (DcMotorEx itemMotor : listMotorsArm) {
            itemMotor.setZeroPowerBehavior(inZeroPowerBehavior);
        }
    }

    public void setArmMotorPower(double inOutputPower) {
        for (DcMotorEx itemMotor : listMotorsArm) {
            itemMotor.setPower(inOutputPower);
        }

    }

    public void setIntakeMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior inZeroPowerBehavior) {
        for (DcMotorEx itemMotor : listMotorsIntake) {
            itemMotor.setZeroPowerBehavior(inZeroPowerBehavior);
        }
    }

    public void setIntakeMotorPower(double inOutputPower) {
        for (DcMotorEx itemMotor : listMotorsIntake) {
            itemMotor.setPower(inOutputPower);
        }

    }
}
