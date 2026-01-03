package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * AUTONOMOUS MODE
 * 1. Reverse a set distance
 * 2. Auto-align to AprilTag
 * 3. Shoot 2-ball cycle
 */
@Autonomous(name = "SimpleReverseAutonomous", group = "Autonomous")
public class SimpleReverseAutonomous extends OpMode {

    // ========================================================================
    // TUNING PARAMETERS - ADJUST THESE VALUES
    // ========================================================================

    // AUTONOMOUS SEQUENCE TUNING
    private final double REVERSE_POWER = -0.3;         // Power while reversing (negative = backward)
    private final int REVERSE_TIME_MS = 3000;          // How long to reverse (milliseconds)
    private final int SETTLE_AFTER_REVERSE_MS = 500;   // Pause before starting alignment

    // STAGE 1: Initial Push Servo (170 degrees movement)
    private final double STAGE1_PUSH_POS = 1;
    private final double STAGE1_RESET_POS = 0.4;

    // STAGE 2: Mid Catch and Push Servo (160 degrees movement)
    private final double STAGE2_PUSH_POS = 0.3;
    private final double STAGE2_RESET_POS = 0.83;

    // STAGE 3: Final Feed Servo (pushes ball to flywheels)
    private final double STAGE3_PUSH_POS = 0.5;
    private final double STAGE3_RESET_POS = 0.25;

    // FLYWHEELS
    private final double FLYWHEEL_SPEED = 0.8;
    private final int FLYWHEEL_SPINUP_MS = 1000;

    // APRILTAG ALIGNMENT
    private final double ALIGN_KP = 0.03;
    private final double ALIGN_TOLERANCE = 3.0;
    private final double ALIGN_MAX_SPEED = 0.1;
    private final double ALIGN_MIN_SPEED = 0.02;
    private final double ALIGN_DEADBAND = 2.0;
    private final double ALIGN_ANGLE_OFFSET = 0.0;
    private final int ALIGN_TIMEOUT_MS = 4000;         // Give up if can't align in 4 seconds

    // AUTO CYCLE TIMING
    private final int BALL1_STAGE1_PUSH_MS = 700;
    private final int BALL1_STAGE1_TO_STAGE2_MS = 10;
    private final int BALL1_STAGE2_PUSH_MS = 800;
    private final int BALL1_STAGE2_TO_STAGE3_MS = 700;
    private final int BALL1_STAGE3_PUSH_MS = 900;
    private final int BALL1_TO_BALL2_DELAY_MS = 200;
    private final int BALL2_STAGE1_PUSH_MS = 700;
    private final int BALL2_STAGE1_TO_STAGE2_MS = 100;
    private final int BALL2_STAGE2_PUSH_MS = 800;
    private final int BALL2_STAGE2_TO_STAGE3_MS = 700;
    private final int BALL2_STAGE3_PUSH_MS = 900;

    // ========================================================================
    // HARDWARE
    // ========================================================================
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor flyLeft, flyRight;
    Servo stage1Servo, stage2Servo, stage3Servo;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection targetTag = null;

    // State machine
    private enum AutoState {
        INIT,
        REVERSING,
        SETTLE_AFTER_REVERSE,
        FINDING_TAG,
        ALIGNING,
        ALIGNED,
        SHOOTING,
        COMPLETE
    }

    private AutoState currentState = AutoState.INIT;
    private long stateStartTime = 0;

    // Shooting state machine
    private enum ShootState {
        IDLE,
        BALL1_STAGE1_PUSH,
        BALL1_STAGE1_RESET,
        BALL1_WAIT_STAGE2,
        BALL1_STAGE2_PUSH,
        BALL1_STAGE2_RESET,
        BALL1_WAIT_STAGE3,
        BALL1_STAGE3_PUSH,
        BALL1_STAGE3_RESET,
        BALL2_WAIT,
        BALL2_STAGE1_PUSH,
        BALL2_STAGE1_RESET,
        BALL2_WAIT_STAGE2,
        BALL2_STAGE2_PUSH,
        BALL2_STAGE2_RESET,
        BALL2_WAIT_STAGE3,
        BALL2_STAGE3_PUSH,
        BALL2_STAGE3_RESET,
        COMPLETE
    }

    private ShootState shootState = ShootState.IDLE;
    private long shootStateStartTime = 0;
    private boolean flywheelsRunning = false;
    private long flywheelStartTime = 0;

    @Override
    public void init() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        flyLeft = hardwareMap.get(DcMotor.class, "fly_left");
        flyRight = hardwareMap.get(DcMotor.class, "fly_right");

        // Initialize servos
        stage1Servo = hardwareMap.get(Servo.class, "stage1");
        stage2Servo = hardwareMap.get(Servo.class, "stage2");
        stage3Servo = hardwareMap.get(Servo.class, "stage3");

        // Motor directions
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        flyRight.setDirection(DcMotor.Direction.REVERSE);
        flyLeft.setDirection(DcMotor.Direction.REVERSE);

        // Brake mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        stage1Servo.setPosition(STAGE1_RESET_POS);
        stage2Servo.setPosition(STAGE2_RESET_POS);
        stage3Servo.setPosition(STAGE3_RESET_POS);

        // Initialize AprilTag detection
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new android.util.Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        telemetry.addLine("=== AUTONOMOUS READY ===");
        telemetry.addLine("Sequence: Reverse -> Align -> Shoot");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start the autonomous sequence
        currentState = AutoState.REVERSING;
        stateStartTime = System.currentTimeMillis();
        telemetry.addLine(">>> AUTONOMOUS STARTED <<<");
        telemetry.update();
    }

    @Override
    public void loop() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        telemetry.addLine("=== AUTONOMOUS MODE ===");
        telemetry.addData("Current State", currentState.toString());
        telemetry.addData("Time in State", "%.1f sec", elapsed / 1000.0);
        telemetry.addLine("");

        switch (currentState) {
            case INIT:
                // Waiting for start
                break;

            case REVERSING:
                // Drive backward
                setDrivePower(REVERSE_POWER, 0, 0);

                if (elapsed >= REVERSE_TIME_MS) {
                    setDrivePower(0, 0, 0);
                    transitionToState(AutoState.SETTLE_AFTER_REVERSE);
                }

                telemetry.addLine(">>> REVERSING <<<");
                break;

            case SETTLE_AFTER_REVERSE:
                // Brief pause to stabilize
                setDrivePower(0, 0, 0);

                if (elapsed >= SETTLE_AFTER_REVERSE_MS) {
                    transitionToState(AutoState.FINDING_TAG);
                }

                telemetry.addLine(">>> SETTLING <<<");
                break;

            case FINDING_TAG:
                // Look for AprilTag
                targetTag = findClosestAprilTag();

                if (targetTag != null) {
                    telemetry.addLine(">>> TAG FOUND: " + targetTag.id + " <<<");
                    transitionToState(AutoState.ALIGNING);
                } else if (elapsed >= 2000) {
                    // Timeout after 2 seconds
                    telemetry.addLine(">>> NO TAG FOUND - SKIPPING TO SHOOT <<<");
                    transitionToState(AutoState.SHOOTING);
                }

                telemetry.addLine(">>> SEARCHING FOR TAG <<<");
                telemetry.addData("Tags Visible", aprilTag.getDetections().size());
                break;

            case ALIGNING:
                // Auto-align to tag
                boolean aligned = performAlignment();

                if (aligned) {
                    setDrivePower(0, 0, 0);
                    telemetry.addLine(">>> ALIGNMENT COMPLETE <<<");
                    transitionToState(AutoState.ALIGNED);
                } else if (elapsed >= ALIGN_TIMEOUT_MS) {
                    setDrivePower(0, 0, 0);
                    telemetry.addLine(">>> ALIGNMENT TIMEOUT - PROCEEDING <<<");
                    transitionToState(AutoState.SHOOTING);
                }
                break;

            case ALIGNED:
                // Brief pause after alignment
                setDrivePower(0, 0, 0);

                if (elapsed >= 500) {
                    transitionToState(AutoState.SHOOTING);
                }

                telemetry.addLine(">>> READY TO SHOOT <<<");
                break;

            case SHOOTING:
                // Execute shooting sequence
                handleShooting();

                if (shootState == ShootState.COMPLETE) {
                    transitionToState(AutoState.COMPLETE);
                }

                telemetry.addLine(">>> SHOOTING <<<");
                telemetry.addData("Shoot State", shootState.toString());
                break;

            case COMPLETE:
                // All done
                setDrivePower(0, 0, 0);
                flyLeft.setPower(0);
                flyRight.setPower(0);

                telemetry.addLine(">>> AUTONOMOUS COMPLETE <<<");
                break;
        }

        // Flywheel status
        handleFlywheelSpinup();
        if (flywheelsRunning) {
            long flywheelElapsed = System.currentTimeMillis() - flywheelStartTime;
            double percent = Math.min(100.0, (double)flywheelElapsed / FLYWHEEL_SPINUP_MS * 100.0);
            telemetry.addData("Flywheels", "%.0f%%", percent);
        }

        telemetry.update();
    }

    // ========================================================================
    // ALIGNMENT LOGIC
    // ========================================================================
    private boolean performAlignment() {
        // Try to find the tag
        targetTag = findTagById(targetTag.id);

        // Lost sight of tag
        if (targetTag == null) {
            telemetry.addLine("Lost tag during alignment");
            setDrivePower(0, 0, 0);
            return false;
        }

        // Get bearing angle
        double bearingRadians = targetTag.ftcPose.bearing;
        double bearingDegrees = Math.toDegrees(bearingRadians) + ALIGN_ANGLE_OFFSET;

        // Check if aligned
        if (Math.abs(bearingDegrees) < ALIGN_TOLERANCE) {
            return true;
        }

        // Don't move if error is tiny
        if (Math.abs(bearingDegrees) < ALIGN_DEADBAND) {
            setDrivePower(0, 0, 0);
            telemetry.addData("Aligning", "Fine adjustment");
            return false;
        }

        // Calculate turn speed
        double turnSpeed = -bearingDegrees * ALIGN_KP;

        // Clamp to max speed
        if (Math.abs(turnSpeed) > ALIGN_MAX_SPEED) {
            turnSpeed = Math.signum(turnSpeed) * ALIGN_MAX_SPEED;
        }

        // Ensure minimum speed
        if (Math.abs(turnSpeed) < ALIGN_MIN_SPEED && Math.abs(turnSpeed) > 0) {
            turnSpeed = Math.signum(turnSpeed) * ALIGN_MIN_SPEED;
        }

        // Turn only
        setDrivePower(0, 0, turnSpeed);

        telemetry.addData("Bearing Error", "%.1f deg", bearingDegrees);
        telemetry.addData("Turn Speed", "%.2f", turnSpeed);
        telemetry.addData("Distance", "%.1f inches", targetTag.ftcPose.range);

        return false;
    }

    private AprilTagDetection findClosestAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return null;

        AprilTagDetection closest = null;
        double closestDist = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose != null && detection.ftcPose.range < closestDist) {
                closestDist = detection.ftcPose.range;
                closest = detection;
            }
        }
        return closest;
    }

    private AprilTagDetection findTagById(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == id && detection.ftcPose != null) {
                return detection;
            }
        }
        return null;
    }

    // ========================================================================
    // SHOOTING LOGIC
    // ========================================================================
    private void handleShooting() {
        if (shootState == ShootState.IDLE) {
            // Start shooting sequence
            shootState = ShootState.BALL1_STAGE1_PUSH;
            shootStateStartTime = System.currentTimeMillis();

            // Start flywheels
            flywheelsRunning = true;
            flywheelStartTime = System.currentTimeMillis();
        }

        updateShootingStateMachine();
    }

    private void updateShootingStateMachine() {
        long elapsed = System.currentTimeMillis() - shootStateStartTime;

        switch (shootState) {
            case BALL1_STAGE1_PUSH:
                stage1Servo.setPosition(STAGE1_PUSH_POS);
                if (elapsed >= BALL1_STAGE1_PUSH_MS) {
                    transitionShootState(ShootState.BALL1_STAGE1_RESET);
                }
                break;

            case BALL1_STAGE1_RESET:
                stage1Servo.setPosition(STAGE1_RESET_POS);
                transitionShootState(ShootState.BALL1_WAIT_STAGE2);
                break;

            case BALL1_WAIT_STAGE2:
                if (elapsed >= BALL1_STAGE1_TO_STAGE2_MS) {
                    transitionShootState(ShootState.BALL1_STAGE2_PUSH);
                }
                break;

            case BALL1_STAGE2_PUSH:
                stage2Servo.setPosition(STAGE2_PUSH_POS);
                if (elapsed >= BALL1_STAGE2_PUSH_MS) {
                    transitionShootState(ShootState.BALL1_STAGE2_RESET);
                }
                break;

            case BALL1_STAGE2_RESET:
                stage2Servo.setPosition(STAGE2_RESET_POS);
                transitionShootState(ShootState.BALL1_WAIT_STAGE3);
                break;

            case BALL1_WAIT_STAGE3:
                if (elapsed >= BALL1_STAGE2_TO_STAGE3_MS) {
                    transitionShootState(ShootState.BALL1_STAGE3_PUSH);
                }
                break;

            case BALL1_STAGE3_PUSH:
                stage3Servo.setPosition(STAGE3_PUSH_POS);
                if (elapsed >= BALL1_STAGE3_PUSH_MS) {
                    transitionShootState(ShootState.BALL1_STAGE3_RESET);
                }
                break;

            case BALL1_STAGE3_RESET:
                stage3Servo.setPosition(STAGE3_RESET_POS);
                transitionShootState(ShootState.BALL2_WAIT);
                break;

            case BALL2_WAIT:
                if (elapsed >= BALL1_TO_BALL2_DELAY_MS) {
                    transitionShootState(ShootState.BALL2_STAGE1_PUSH);
                }
                break;

            case BALL2_STAGE1_PUSH:
                stage1Servo.setPosition(STAGE1_PUSH_POS);
                if (elapsed >= BALL2_STAGE1_PUSH_MS) {
                    transitionShootState(ShootState.BALL2_STAGE1_RESET);
                }
                break;

            case BALL2_STAGE1_RESET:
                stage1Servo.setPosition(STAGE1_RESET_POS);
                transitionShootState(ShootState.BALL2_WAIT_STAGE2);
                break;

            case BALL2_WAIT_STAGE2:
                if (elapsed >= BALL2_STAGE1_TO_STAGE2_MS) {
                    transitionShootState(ShootState.BALL2_STAGE2_PUSH);
                }
                break;

            case BALL2_STAGE2_PUSH:
                stage2Servo.setPosition(STAGE2_PUSH_POS);
                if (elapsed >= BALL2_STAGE2_PUSH_MS) {
                    transitionShootState(ShootState.BALL2_STAGE2_RESET);
                }
                break;

            case BALL2_STAGE2_RESET:
                stage2Servo.setPosition(STAGE2_RESET_POS);
                transitionShootState(ShootState.BALL2_WAIT_STAGE3);
                break;

            case BALL2_WAIT_STAGE3:
                if (elapsed >= BALL2_STAGE2_TO_STAGE3_MS) {
                    transitionShootState(ShootState.BALL2_STAGE3_PUSH);
                }
                break;

            case BALL2_STAGE3_PUSH:
                stage3Servo.setPosition(STAGE3_PUSH_POS);
                if (elapsed >= BALL2_STAGE3_PUSH_MS) {
                    transitionShootState(ShootState.BALL2_STAGE3_RESET);
                }
                break;

            case BALL2_STAGE3_RESET:
                stage3Servo.setPosition(STAGE3_RESET_POS);
                transitionShootState(ShootState.COMPLETE);
                break;

            case COMPLETE:
                // Shooting complete
                flywheelsRunning = false;
                flyLeft.setPower(0);
                flyRight.setPower(0);
                break;
        }
    }

    private void transitionShootState(ShootState newState) {
        shootState = newState;
        shootStateStartTime = System.currentTimeMillis();
    }

    // ========================================================================
    // HELPER FUNCTIONS
    // ========================================================================
    private void transitionToState(AutoState newState) {
        currentState = newState;
        stateStartTime = System.currentTimeMillis();
    }

    private void setDrivePower(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;
        double backLeftPower = forward - strafe + rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower))
        ));

        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }

    private void handleFlywheelSpinup() {
        if (flywheelsRunning) {
            long elapsed = System.currentTimeMillis() - flywheelStartTime;
            double power = Math.min(1.0, (double)elapsed / FLYWHEEL_SPINUP_MS) * FLYWHEEL_SPEED;
            flyLeft.setPower(power);
            flyRight.setPower(power);
        } else {
            flyLeft.setPower(0);
            flyRight.setPower(0);
        }
    }
}