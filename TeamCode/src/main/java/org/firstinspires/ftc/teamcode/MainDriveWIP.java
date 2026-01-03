package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * TWO-BALL AUTO CYCLE with AprilTag Alignment
 *
 * CONTROLS:
 * RIGHT TRIGGER:         Full 2-Ball Auto Cycle
 * B button:              Auto-align to nearest AprilTag
 * Intake:                A = IN | X = OUT
 *
 * Drive:                 Left stick = move, Right stick X = rotate
 */
@TeleOp(name = "Main Drive WIP", group = "Robot")
public class MainDriveWIP extends OpMode {

    // ========================================================================
    // TUNING PARAMETERS - ADJUST THESE VALUES
    // ========================================================================

    // STAGE 1: Initial Push Servo (170 degrees movement)
    private final double STAGE1_PUSH_POS = 1;
    private final double STAGE1_RESET_POS = 0.4;

    // STAGE 2: Mid Catch and Push Servo (160 degrees movement)
    private final double STAGE2_PUSH_POS = 0.3;
    private final double STAGE2_RESET_POS = 1;

    // STAGE 3: Final Feed Servo (pushes ball to flywheels)
    private final double STAGE3_PUSH_POS = 0.5;
    private final double STAGE3_RESET_POS = 0.23;

    // FLYWHEELS
    private final double FLYWHEEL_SPEED = 0.8;
    private final int FLYWHEEL_SPINUP_MS = 500;

    // INTAKE
    private final double INTAKE_SPEED = 0.8;
    private final double OUTTAKE_SPEED = 0.6;

    // APRILTAG ALIGNMENT - FIXED PARAMETERS
    private final double ALIGN_KP = 0.03;              // Proportional gain (increased for more responsive turning)
    private final double ALIGN_TOLERANCE = 3.0;        // Degrees - within this = aligned
    private final double ALIGN_MAX_SPEED = 0.08;        // Maximum turn speed
    private final double ALIGN_MIN_SPEED = 0.02;       // Minimum turn speed (helps with small adjustments)
    private final double ALIGN_DEADBAND = 2.0;         // Below this error, stop completely
    private final double ALIGN_ANGLE_OFFSET = 0.0;     // Offset if you need to aim left/right of center

    // ========================================================================
    // AUTO CYCLE TIMING - Right Trigger 2-Ball Sequence
    // ========================================================================

    // BALL 1 TIMING
    private final int BALL1_STAGE1_PUSH_MS = 700;
    private final int BALL1_STAGE1_TO_STAGE2_MS = 10;
    private final int BALL1_STAGE2_PUSH_MS = 800;
    private final int BALL1_STAGE2_TO_STAGE3_MS = 700;
    private final int BALL1_STAGE3_PUSH_MS = 900;

    // BALL 2 TIMING
    private final int BALL1_TO_BALL2_DELAY_MS = 200;
    private final int BALL2_STAGE1_PUSH_MS = 700;
    private final int BALL2_STAGE1_TO_STAGE2_MS = 100;
    private final int BALL2_STAGE2_PUSH_MS = 800;
    private final int BALL2_STAGE2_TO_STAGE3_MS = 700;
    private final int BALL2_STAGE3_PUSH_MS = 900;

    // AUTO CYCLE BEHAVIOR
    private final boolean AUTO_START_FLYWHEELS = true;
    private final boolean AUTO_RESET_AFTER_CYCLE = true;

    // ========================================================================
    // HARDWARE
    // ========================================================================
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intakeMotor, flyLeft, flyRight;

    Servo stage1Servo;
    Servo stage2Servo;
    Servo stage3Servo;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean isAligning = false;
    private AprilTagDetection targetTag = null;

    // Flywheel control
    private boolean flywheelsRunning = false;
    private long flywheelStartTime = 0;

    // Auto cycle state machine
    private enum AutoCycleState {
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

    private AutoCycleState autoCycleState = AutoCycleState.IDLE;
    private long autoCycleStateStartTime = 0;
    private boolean lastRightTrigger = false;

    @Override
    public void init() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        flyLeft = hardwareMap.get(DcMotor.class, "fly_left");
        flyRight = hardwareMap.get(DcMotor.class, "fly_right");

        // Initialize servos
        stage1Servo = hardwareMap.get(Servo.class, "stage1");
        stage2Servo = hardwareMap.get(Servo.class, "stage2");
        stage3Servo = hardwareMap.get(Servo.class, "stage3");

        // Motor directions
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyRight.setDirection(DcMotor.Direction.REVERSE);
        flyLeft.setDirection(DcMotor.Direction.REVERSE);

        // Brake mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        telemetry.addLine("=== SYSTEM READY ===");
        telemetry.addLine("Right Trigger: Auto Cycle");
        telemetry.addLine("B Button: AprilTag Align");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("=== TWO-BALL AUTO CYCLE ===");
        telemetry.addLine("");

        // Handle all controls
        handleAutoCycle();
        handleIntakeControl();
        handleDriving();
        handleAprilTagAlign();
        handleFlywheelSpinup();
        handleStage3Manual();

        // Display status
        displayStatus();

        telemetry.update();
    }
    

    // ========================================================================
    // AUTO CYCLE CONTROL
    // ========================================================================
    private void handleAutoCycle() {
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;

        // Start cycle
        if (rightTriggerPressed && !lastRightTrigger && autoCycleState == AutoCycleState.IDLE) {
            startAutoCycle();
        }

        // Emergency stop
        if (!rightTriggerPressed && autoCycleState != AutoCycleState.IDLE && autoCycleState != AutoCycleState.COMPLETE) {
            stopAutoCycle();
        }

        lastRightTrigger = rightTriggerPressed;

        // Update state machine
        if (autoCycleState != AutoCycleState.IDLE) {
            updateAutoCycle();
        }
    }

    private void startAutoCycle() {
        autoCycleState = AutoCycleState.BALL1_STAGE1_PUSH;
        autoCycleStateStartTime = System.currentTimeMillis();

        if (AUTO_START_FLYWHEELS && !flywheelsRunning) {
            flywheelsRunning = true;
            flywheelStartTime = System.currentTimeMillis();
        }
    }

    private void stopAutoCycle() {
        autoCycleState = AutoCycleState.IDLE;
        stage1Servo.setPosition(STAGE1_RESET_POS);
        stage2Servo.setPosition(STAGE2_RESET_POS);
        stage3Servo.setPosition(STAGE3_RESET_POS);

        // Stop flywheels when trigger is released
        flywheelsRunning = false;
        flyLeft.setPower(0);
        flyRight.setPower(0);
    }

    private void updateAutoCycle() {
        long elapsed = System.currentTimeMillis() - autoCycleStateStartTime;

        switch (autoCycleState) {
            case BALL1_STAGE1_PUSH:
                stage1Servo.setPosition(STAGE1_PUSH_POS);
                if (elapsed >= BALL1_STAGE1_PUSH_MS) {
                    transitionAutoCycle(AutoCycleState.BALL1_STAGE1_RESET);
                }
                break;

            case BALL1_STAGE1_RESET:
                stage1Servo.setPosition(STAGE1_RESET_POS);
                transitionAutoCycle(AutoCycleState.BALL1_WAIT_STAGE2);
                break;

            case BALL1_WAIT_STAGE2:
                if (elapsed >= BALL1_STAGE1_TO_STAGE2_MS) {
                    transitionAutoCycle(AutoCycleState.BALL1_STAGE2_PUSH);
                }
                break;

            case BALL1_STAGE2_PUSH:
                stage2Servo.setPosition(STAGE2_PUSH_POS);
                if (elapsed >= BALL1_STAGE2_PUSH_MS) {
                    transitionAutoCycle(AutoCycleState.BALL1_STAGE2_RESET);
                }
                break;

            case BALL1_STAGE2_RESET:
                stage2Servo.setPosition(STAGE2_RESET_POS);
                transitionAutoCycle(AutoCycleState.BALL1_WAIT_STAGE3);
                break;

            case BALL1_WAIT_STAGE3:
                if (elapsed >= BALL1_STAGE2_TO_STAGE3_MS) {
                    transitionAutoCycle(AutoCycleState.BALL1_STAGE3_PUSH);
                }
                break;

            case BALL1_STAGE3_PUSH:
                stage3Servo.setPosition(STAGE3_PUSH_POS);
                if (elapsed >= BALL1_STAGE3_PUSH_MS) {
                    transitionAutoCycle(AutoCycleState.BALL1_STAGE3_RESET);
                }
                break;

            case BALL1_STAGE3_RESET:
                stage3Servo.setPosition(STAGE3_RESET_POS);
                transitionAutoCycle(AutoCycleState.BALL2_WAIT);
                break;

            case BALL2_WAIT:
                if (elapsed >= BALL1_TO_BALL2_DELAY_MS) {
                    transitionAutoCycle(AutoCycleState.BALL2_STAGE1_PUSH);
                }
                break;

            case BALL2_STAGE1_PUSH:
                stage1Servo.setPosition(STAGE1_PUSH_POS);
                if (elapsed >= BALL2_STAGE1_PUSH_MS) {
                    transitionAutoCycle(AutoCycleState.BALL2_STAGE1_RESET);
                }
                break;

            case BALL2_STAGE1_RESET:
                stage1Servo.setPosition(STAGE1_RESET_POS);
                transitionAutoCycle(AutoCycleState.BALL2_WAIT_STAGE2);
                break;

            case BALL2_WAIT_STAGE2:
                if (elapsed >= BALL2_STAGE1_TO_STAGE2_MS) {
                    transitionAutoCycle(AutoCycleState.BALL2_STAGE2_PUSH);
                }
                break;

            case BALL2_STAGE2_PUSH:
                stage2Servo.setPosition(STAGE2_PUSH_POS);
                if (elapsed >= BALL2_STAGE2_PUSH_MS) {
                    transitionAutoCycle(AutoCycleState.BALL2_STAGE2_RESET);
                }
                break;

            case BALL2_STAGE2_RESET:
                stage2Servo.setPosition(STAGE2_RESET_POS);
                transitionAutoCycle(AutoCycleState.BALL2_WAIT_STAGE3);
                break;

            case BALL2_WAIT_STAGE3:
                if (elapsed >= BALL2_STAGE2_TO_STAGE3_MS) {
                    transitionAutoCycle(AutoCycleState.BALL2_STAGE3_PUSH);
                }
                break;

            case BALL2_STAGE3_PUSH:
                stage3Servo.setPosition(STAGE3_PUSH_POS);
                if (elapsed >= BALL2_STAGE3_PUSH_MS) {
                    transitionAutoCycle(AutoCycleState.BALL2_STAGE3_RESET);
                }
                break;

            case BALL2_STAGE3_RESET:
                stage3Servo.setPosition(STAGE3_RESET_POS);
                transitionAutoCycle(AutoCycleState.COMPLETE);
                break;

            case COMPLETE:
                if (elapsed >= 500) {
                    if (AUTO_RESET_AFTER_CYCLE) {
                        stage1Servo.setPosition(STAGE1_RESET_POS);
                        stage2Servo.setPosition(STAGE2_RESET_POS);
                        stage3Servo.setPosition(STAGE3_RESET_POS);
                    }
                    // Stop flywheels when cycle completes
                    flywheelsRunning = false;
                    flyLeft.setPower(0);
                    flyRight.setPower(0);
                    autoCycleState = AutoCycleState.IDLE;
                }
                break;
        }
    }

    private void transitionAutoCycle(AutoCycleState newState) {
        autoCycleState = newState;
        autoCycleStateStartTime = System.currentTimeMillis();
    }

    // ========================================================================
    // FLYWHEEL CONTROL
    // ========================================================================
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

    

    // ========================================================================
    // INTAKE CONTROL
    // ========================================================================
    private void handleIntakeControl() {
        if (gamepad1.a) {
            intakeMotor.setPower(INTAKE_SPEED);
        } else if (gamepad1.x) {
            intakeMotor.setPower(-OUTTAKE_SPEED);
        } else {
            intakeMotor.setPower(0);
        }
    }
    // ========================================================================
    // STAGE 3 MANUAL OVERRIDE (RIGHT BUMPER HOLD)
    // ========================================================================
    private void handleStage3Manual() {
        // Only allow manual control when auto cycle is idle or complete
        if (autoCycleState == AutoCycleState.IDLE || autoCycleState == AutoCycleState.COMPLETE) {
            if (gamepad1.right_bumper) {
                // While holding right bumper, move to push position
                stage3Servo.setPosition(STAGE3_PUSH_POS);
            } else {
                // When released, return to reset position
                stage3Servo.setPosition(STAGE3_RESET_POS);
            }
        }
    }



    // ========================================================================
    // DRIVING & APRILTAG ALIGNMENT
    // ========================================================================
    private void handleDriving() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Check if driver is moving
        boolean driverIsMoving = Math.abs(forward) > 0.05 ||
                Math.abs(strafe) > 0.05 ||
                Math.abs(rotate) > 0.05;

        // Cancel alignment if driver moves
        if (driverIsMoving && isAligning) {
            isAligning = false;
            telemetry.addLine(">>> You looked ^_^");
        }

        // Drive or align
        if (isAligning) {
            autoAlign();
        } else {
            mecanum(forward, strafe, rotate);
        }
    }

    private void handleAprilTagAlign() {
        // B button starts alignment
        if (gamepad1.b && !isAligning) {
            targetTag = findClosestAprilTag();
            if (targetTag != null) {
                isAligning = true;
                telemetry.addLine(">>> Starting alignment to tag " + targetTag.id);
            } else {
                telemetry.addLine(">>> No AprilTag detected!");
            }
        }
    }

    private void autoAlign() {
        // Try to find the tag we're aligning to
        targetTag = findTagById(targetTag.id);

        // Lost sight of tag
        if (targetTag == null) {
            telemetry.addLine(">>> LOST TAG - Alignment stopped");
            isAligning = false;
            mecanum(0, 0, 0);
            return;
        }

        // Get bearing angle (how far left/right the tag is)
        double bearingRadians = targetTag.ftcPose.bearing;
        double bearingDegrees = Math.toDegrees(bearingRadians) + ALIGN_ANGLE_OFFSET;

        // Check if we're aligned
        if (Math.abs(bearingDegrees) < ALIGN_TOLERANCE) {
            telemetry.addLine(">>> ALIGNED!");
            isAligning = false;
            mecanum(0, 0, 0);
            return;
        }

        // Don't move if error is tiny (prevents jitter)
        if (Math.abs(bearingDegrees) < ALIGN_DEADBAND) {
            mecanum(0, 0, 0);
            telemetry.addData("Aligning", "Fine adjustment - holding");
            return;
        }

        // Calculate turn speed using proportional control (negative to fix direction)
        double turnSpeed = -bearingDegrees * ALIGN_KP;

        // Clamp to max speed
        if (Math.abs(turnSpeed) > ALIGN_MAX_SPEED) {
            turnSpeed = Math.signum(turnSpeed) * ALIGN_MAX_SPEED;
        }

        // Ensure minimum speed for small corrections
        if (Math.abs(turnSpeed) < ALIGN_MIN_SPEED && Math.abs(turnSpeed) > 0) {
            turnSpeed = Math.signum(turnSpeed) * ALIGN_MIN_SPEED;
        }

        // Turn (no forward/strafe, just rotation)
        mecanum(0, 0, turnSpeed);

        telemetry.addData("Aligning to Tag", targetTag.id);
        telemetry.addData("Bearing Error", "%.1f deg", bearingDegrees);
        telemetry.addData("Turn Speed", "%.2f", turnSpeed);
        telemetry.addData("Distance", "%.1f inches", targetTag.ftcPose.range);
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

    private void mecanum(double forward, double strafe, double rotate) {
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

    // ========================================================================
    // TELEMETRY
    // ========================================================================
    private void displayStatus() {
        // Auto cycle status
        if (autoCycleState != AutoCycleState.IDLE) {
            telemetry.addLine("--- AUTO CYCLE ACTIVE ---");
            telemetry.addData("State", autoCycleState.toString());
        }

        // Flywheel status
        if (flywheelsRunning) {
            long elapsed = System.currentTimeMillis() - flywheelStartTime;
            double percent = Math.min(100.0, (double)elapsed / FLYWHEEL_SPINUP_MS * 100.0);
            telemetry.addData("Flywheels", "%.0f%%", percent);
        } else {
            telemetry.addData("Flywheels", "OFF");
        }

        // Intake
        double intakePower = intakeMotor.getPower();
        if (Math.abs(intakePower) > 0.01) {
            telemetry.addData("Intake", intakePower > 0 ? "IN" : "OUT");
        }

        // AprilTag
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("AprilTags Visible", detections.size());
        if (isAligning) {
            telemetry.addLine(">>> ALIGNING <<<");
        }

        telemetry.addLine("");
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Right Trigger: Auto 2-Ball Cycle");
        telemetry.addLine("B: AprilTag Align");
        telemetry.addLine("A: Intake IN | X: Intake OUT");
    }
}