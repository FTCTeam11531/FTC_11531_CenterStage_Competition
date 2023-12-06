package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.system.sysDrivetrainOdometryMecanum;
import org.firstinspires.ftc.teamcode.system.sysIntakeArm;
import org.firstinspires.ftc.teamcode.system.sysLighting;
import org.firstinspires.ftc.teamcode.system.sysVision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

@Config
@Autonomous(name = "Autonomous - Testing", group = "_auto", preselectTeleOp = "Teleop Main")
@Disabled
public class opmodeAutonomousTesting extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initializtion

    // -- Lighting System
    sysLighting sysLighting = new sysLighting(this);

    // -- Drivetrain System
    // defined below

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
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // ------------------------------------------------------------
        // Initialize System(s) - set different light mode between each system init
        // ------------------------------------------------------------

        // Initialize System(s)
        sysLighting.init(utilRobotConstants.Configuration.ENABLE_LIGHTING);
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LIGHTING);

//        sysDrivetrain.init();
        sysDrivetrainOdometryMecanum sysDrivetrain = new sysDrivetrainOdometryMecanum(hardwareMap);
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN);

        sysVision.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

        sysIntakeArm.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_INTAKEARM);

        // ------------------------------------------------------------
        // Configure drivetrain for Autonomous Mode
        // -- Set to run without encoders for timed drive mode
        // ------------------------------------------------------------
//        sysDrivetrain.setDriveMotorRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        boolean isImageFound = false;
        int targetZone = 0, movementAmount = 0;

        Pose2d poseEstimate;

        HuskyLens.Block[] targetObjectList = null;
        HuskyLens.Block targetObject = null;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // Reset runtime clock
        runtime.reset();

        // System Initialization Complete
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_COMPLETE);

        // Repeat while in initialize state (disable if using waitForStart)

        // Set AI Camera Mode
        sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_OBJECT_TRACKING);

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to auto-clear
//        telemetry.setAutoClear(true);
        telemetry.clearAll();

        while(opModeInInit() && !isStopRequested()) {

            // Look for Team Element position
            targetObjectList = sysVision.getCameraObjectList();

            if(targetObjectList.length > 0) {

                targetObject = sysVision.getCameraObject(targetObjectList, 1);

                if(targetObject != null) {

                    if (targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X) {

                        // Set Zone based on Alliance
                        if(sysVision.getDetectedAllianceTagColor() == "blue") {
                            targetZone = 1; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                        }
                        else {
                            targetZone = 3; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                        }
                    } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X && targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {
                        targetZone = 2; // center
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                    } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {

                        // Set Zone based on Alliance
                        if(sysVision.getDetectedAllianceTagColor() == "blue") {
                            targetZone = 3; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                        }
                        else {
                            targetZone = 1; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                        }
                    } else {

                        // Set Zone based on Alliance
                        if(sysVision.getDetectedAllianceTagColor() == "blue") {
                            targetZone = 3; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                        }
                        else {
                            targetZone = 1; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                        }
                    }

                }
            }
            else {

                // Set Zone based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    targetZone = 3; // left
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                }
                else {
                    targetZone = 1; // left
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                }
            }

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Init Time", runtime.toString());
            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Alliance Color", sysVision.getDetectedAllianceTagColor());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Camera Block Count", sysVision.getCameraObjectList().length);
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Randomized Zone", targetZone);
            if (targetObject != null) {
                telemetry.addData("-", "------------------------------");
                telemetry.addData("-", "-- Target Object");
                telemetry.addData("-", "------------------------------");
                telemetry.addData("Target x:", targetObject.x);
                telemetry.addData("Target y:", targetObject.y);
                telemetry.addData("Target width:", targetObject.width);
                telemetry.addData("Target height:", targetObject.height);
                telemetry.addData("Target top:", targetObject.top);
                telemetry.addData("Target left:", targetObject.left);
            }

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
        }

        // Road Runner -- need to implement in system
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Set Starting Pose for Blue Station Two
        Pose2d startPose = new Pose2d(-64, -33, Math.toRadians(0));
        sysDrivetrain.setPoseEstimate(startPose);

        // ----------------------------------------------------
        // Defined Trajectories
        // ----------------------------------------------------
        // TODO: clean up logic
        // ----------------------
        // TESTING!!!!!
        // ----------------------

        TrajectorySequence trajSeqBlueAudienceZone1 = sysDrivetrain.trajectorySequenceBuilder(startPose)
                // Move to Random Zone 1
                .lineToSplineHeading(new Pose2d(-36, -31, Math.toRadians(90)))

                // Place Purple Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Move to Cycle Lane
                .lineToSplineHeading(new Pose2d(-14, -39, Math.toRadians(87)))

                // Move Across Field - to other side
                .lineToSplineHeading(new Pose2d(-14, 48, Math.toRadians(90)))

                // Move to board
                .lineToSplineHeading(new Pose2d(-36, 48, Math.toRadians(90)))

                // Action - Raise Arm
                .addTemporalMarker(() -> {

                    // Raise Arm
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_PRECLIMB, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);

                })
                .waitSeconds(2)

                // Move a little closer to board
                .splineTo(
                        new Vector2d(-36, 52), Math.toRadians(90),
                        sysDrivetrain.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        sysDrivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                // Place Yellow Pixel
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                })
                .waitSeconds(2)

                // Lower Arm
                .addTemporalMarker(() -> {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                })
                .waitSeconds(2)

                // Drive to home
                .lineToSplineHeading(new Pose2d(-14, 48, Math.toRadians(0)))

                .build();



        // Position One
        Trajectory trajZoneOneMoveToZone = sysDrivetrain.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(-36, -31, Math.toRadians(90)))
                .addTemporalMarker(2, () -> {

                    // Place Purple Pixel
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                })
                .build();

        Trajectory trajZoneOneMoveToCycle = sysDrivetrain.trajectoryBuilder(trajZoneOneMoveToZone.end(), false)
                .lineToSplineHeading(new Pose2d(-14, -39, Math.toRadians(87)))
                .build();


        // Position Two
//        Trajectory trajZoneTwoMoveToZoneNew = sysDrivetrain.trajectoryBuilder(startPose, false)
//                .lineToSplineHeading(new Pose2d(-18, -34, Math.toRadians(180)))
//                .addTemporalMarker(2, () -> {
//
//                    // Place Purple Pixel
//                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
//                })
//                .build();
//
//        Trajectory trajZoneTwoMoveToZone = sysDrivetrain.trajectoryBuilder(startPose, false)
//                .forward(36)
//                .addTemporalMarker(2, () -> {
//
//                    // Place Purple Pixel
//                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
//                })
//                .build();

//        Trajectory trajZoneTwoMoveToCycleStageOne = sysDrivetrain.trajectoryBuilder(trajZoneTwoMoveToZone.end(), false)
//                .strafeRight(18)
//                .build();
//
//        Trajectory trajZoneTwoMoveToCycleStageTwo = sysDrivetrain.trajectoryBuilder(trajZoneTwoMoveToCycleStageOne.end(), false)
//                .lineToSplineHeading(new Pose2d(-14, -39, Math.toRadians(87)))
//                .build();



        // Move forward
        Trajectory trajMoveAcrossField = sysDrivetrain.trajectoryBuilder(trajZoneOneMoveToCycle.end(), false)
                .lineToSplineHeading(new Pose2d(-14, 48, Math.toRadians(90)))
                .build();

        // Move to board
        Trajectory trajMovetoBoard = sysDrivetrain.trajectoryBuilder(trajMoveAcrossField.end(), false)
                .lineToSplineHeading(new Pose2d(-36, 48, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {

                        // Raise Arm
                        sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD);
                        sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_PRECLIMB, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);

                })
                .build();

        Trajectory trajPlacePixel = sysDrivetrain.trajectoryBuilder(trajMovetoBoard.end(), false)
                .splineTo(
                        new Vector2d(-36, 52), Math.toRadians(90),
                        sysDrivetrain.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        sysDrivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(1, () -> {

                    // Place Yellow Pixel
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                })
                .build();

        // Move to home
        Trajectory trajMovetoHome = sysDrivetrain.trajectoryBuilder(trajMovetoBoard.end(), false)
                .addTemporalMarker(1, () -> {

                    // Lower Arm
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME);
                    sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                })
                .lineToSplineHeading(new Pose2d(-14, 48, Math.toRadians(0)))
                .build();


        // ----------------------
        // Same for ALL
        // ----------------------
        // Move to Random Zone
        Trajectory trajRandomZone = sysDrivetrain.trajectoryBuilder(startPose, false)
                .forward(24)
                .build();

        // Determine Zone Angle
        switch (targetZone){
            case 1:
                // Zone 1
                movementAmount = -5;

                // Set Color to zone 1
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                break;
            case 2:
                // Zone 2
                movementAmount = -5;

                // Set Color to zone 2
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                break;
            case 3:
                // Zone 3
                movementAmount = -2;

                // Set Color to zone 3
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                break;
            default:
                // Default
                movementAmount = -5;

                // Set Color to invalid when default
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID);
                break;
        }

        // Clear Pixel
        Trajectory trajClearPixel = sysDrivetrain.trajectoryBuilder(trajRandomZone.end(), false)
                .forward(movementAmount)
                .build();

        // ----------------------
        // Alliance Movement
        // ----------------------
        // Strafe out of random zone
        // Strafe based on Alliance and Zone

        // Determine Zone Angle
        switch (targetZone){
            case 1:
                // Zone 1
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 24;
                }
                else {
                    movementAmount = -24;
                }

                // Set Color to zone 1
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                break;
            case 2:
                // Zone 2
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 24;
                }
                else {
                    movementAmount = -24;
                }

                // Set Color to zone 2
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                break;
            case 3:
                // Zone 3
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 1;
                }
                else {
                    movementAmount = -1;
                }

                // Set Color to zone 3
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                break;
            default:
                // Default
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 24;
                }
                else {
                    movementAmount = -24;
                }

                // Set Color to invalid when default
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID);
                break;
        }

        Trajectory trajClearRandomZone = sysDrivetrain.trajectoryBuilder(trajClearPixel.end(), false)
                .strafeRight(movementAmount)
                .build();

        // ----------------------
        // Same for ALL
        // ----------------------
        // Move into cycle lane - from lower level

        // Determine Zone Angle
        switch (targetZone){
            case 1:
                // Zone 1
                movementAmount = 20;

                // Set Color to zone 1
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                break;
            case 2:
                // Zone 2
                movementAmount = 24;

                // Set Color to zone 2
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                break;
            case 3:
                // Zone 3
                movementAmount = 20;

                // Set Color to zone 3
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                break;
            default:
                // Default
                movementAmount = 20;

                // Set Color to invalid when default
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID);
                break;
        }
        Trajectory trajEnterCycleLaneLower = sysDrivetrain.trajectoryBuilder(trajClearRandomZone.end(), false)
                .forward(movementAmount)
                .build();

        // Move to stage
        Trajectory trajMovetoStage = sysDrivetrain.trajectoryBuilder(trajEnterCycleLaneLower.end(), false)
                .forward(92) //92
                .build();

        // Set Turn Angle based on Alliance
        if(sysVision.getDetectedAllianceTagColor() == "blue") {
            movementAmount = -24;
        }
        else {
            movementAmount = 24;
        }

        // Move to park
        Trajectory trajStrafeToPark = sysDrivetrain.trajectoryBuilder(trajMovetoStage.end())
                .strafeRight(movementAmount)
                .build();

        // Wait for Start state (disable if using opModeInInit)
        waitForStart();

        if (isStopRequested()) return;

        // ----------------------------------------------------
        // Movement Action(s)
        // ----------------------------------------------------
        // ----------------------
        // Same for ALL
        // ----------------------


        // Blue - Audience - Zone 1
        sysDrivetrain.followTrajectorySequence(trajSeqBlueAudienceZone1);

        // Move to Zone One
//        sysDrivetrain.followTrajectory(trajZoneOneMoveToZone);
//
//        sysDrivetrain.followTrajectory(trajZoneOneMoveToCycle);
//
//        sysDrivetrain.followTrajectory(trajMoveAcrossField);
//
//        sysDrivetrain.followTrajectory(trajMovetoBoard);
//
//        sysDrivetrain.followTrajectory(trajPlacePixel);
//
//        sysDrivetrain.followTrajectory(trajMovetoHome);

        // Move to Zone Two
//        sysDrivetrain.followTrajectory(trajZoneTwoMoveToZone);
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("-", "");
//        telemetry.addData("-", "------------------------------");
//        telemetry.addData("-", "Zone Two - Move to Zone");
//        telemetry.addData("-", "------------------------------");
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();

//        sysDrivetrain.followTrajectory(trajZoneTwoMoveToCycleStageOne);
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("-", "");
//        telemetry.addData("-", "------------------------------");
//        telemetry.addData("-", "Zone Two - Move to Cycle Stage One");
//        telemetry.addData("-", "------------------------------");
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();

//        sysDrivetrain.followTrajectory(trajZoneTwoMoveToCycleStageTwo);
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("-", "");
//        telemetry.addData("-", "------------------------------");
//        telemetry.addData("-", "Zone Two - Move to Cycle Stage Two");
//        telemetry.addData("-", "------------------------------");
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();









//        // Determine Zone Angle
//        switch (targetZone){
//            case 1:
//                // Zone 1
//                // Set Turn Angle based on Alliance
//                if(sysVision.getDetectedAllianceTagColor() == "blue") {
//                    movementAmount = 74;
//                }
//                else {
//                    movementAmount = -74;
//                }
//
//                // Set Color to zone 1
//                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
//                break;
//            case 2:
//                // Zone 2
//                // Set Turn Angle based on Alliance
//                if(sysVision.getDetectedAllianceTagColor() == "blue") {
//                    movementAmount = 0;
//                }
//                else {
//                    movementAmount = 0;
//                }
//
//                // Set Color to zone 2
//                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
//                break;
//            case 3:
//                // Zone 3
//                // Set Turn Angle based on Alliance
//                if(sysVision.getDetectedAllianceTagColor() == "blue") {
//                    movementAmount = -74;
//                }
//                else {
//                    movementAmount = 74;
//                }
//
//                // Set Color to zone 3
//                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
//                break;
//            default:
//                // Default
//                // Set Turn Angle based on Alliance
//                if(sysVision.getDetectedAllianceTagColor() == "blue") {
//                    movementAmount = 74;
//                }
//                else {
//                    movementAmount = -74;
//                }
//
//                // Set Color to invalid when default
//                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID);
//                break;
//        }

//        // Turn to Zone
//        sysDrivetrain.turn(Math.toRadians(movementAmount));
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();

//        // Place Purple Pixel
//        sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
//        sleep(300);

//        // Clear the Pixel
//        sysDrivetrain.followTrajectory(trajClearPixel);
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();
//
//        // Zone Counter-Turn
//        sysDrivetrain.turn(Math.toRadians(-(movementAmount)));
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();
//
//        // Clear the Random Zone
//        sysDrivetrain.followTrajectory(trajClearRandomZone);
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();
//
//        // Move into Cycle Lane
//        sysDrivetrain.followTrajectory(trajEnterCycleLaneLower);
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();

//        // Turn to Stage
//        // Set Turn Angle based on Alliance
//        if(sysVision.getDetectedAllianceTagColor() == "blue") {
//            movementAmount = 72;
//        }
//        else {
//            movementAmount = -78;
//        }
//
//        sysDrivetrain.turn(Math.toRadians(movementAmount));
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();
//
//        // ----------------------
//        // Same for ALL
//        // ----------------------
//        // Move to stage
//        sysDrivetrain.followTrajectory(trajMovetoStage);
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();
//
//        // Final Turn - In Park
//        // Set Turn Angle based on Alliance
//        if(sysVision.getDetectedAllianceTagColor() == "blue") {
//            movementAmount = -72;
//        }
//        else {
//            movementAmount = 72;
//        }
//
//        sysDrivetrain.turn(Math.toRadians(movementAmount));
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();
//
//        // Move to Park
//        sysDrivetrain.followTrajectory(trajStrafeToPark);
//
//        poseEstimate = sysDrivetrain.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();

        while (!isStopRequested() && opModeIsActive());

    }


}
