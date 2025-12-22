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
 * MANUAL INDEPENDENT CONTROL MODE
 * Each servo controlled individually with dedicated buttons
 * Perfect for testing and tuning each stage
 *
 * CONTROLS:
 * Stage 1 (170° pusher): D-pad Up = PUSH | D-pad Down = RESET
 * Stage 2 (160° pusher): D-pad Left = PUSH | D-pad Right = RESET
 * Stage 3 (Feeder):      Y = PUSH | A = RESET
 * Flywheels:             Right Bumper = ON/OFF toggle
 * Intake:                Left Bumper = IN | X = OUT
 *
 * Drive:                 Left stick = move, Right stick X = rotate
 * AprilTag Align:        B button
 */
@TeleOp(name = "Main Drive WIP", group = "Robot")
public class MainDriveWIP extends OpMode {

    // ========================================================================
    // TUNING PARAMETERS - ADJUST THESE VALUES
    // ========================================================================

    // STAGE 1: Initial Push Servo (170 degrees movement)
    private final double STAGE1_PUSH_POS = 1;
    private final double STAGE1_RESET_POS = 0.4; // Reset/idle position

    // STAGE 2: Mid Catch and Push Servo (160 degrees movement)
    private final double STAGE2_PUSH_POS = 0.3;       // Push position (0-1, ~160° = 0.89)
    private final double STAGE2_RESET_POS = 0.83;      // Reset/catch position

    // STAGE 3: Final Feed Servo (pushes ball to flywheels)
    private final double STAGE3_PUSH_POS = 0.5;       // Feed/push position
    private final double STAGE3_RESET_POS = 0.25;    // Reset/idle position

    // FLYWHEELS
    private final double FLYWHEEL_SPEED = 0.6;       // Max flywheel power (0-1)
    private final int FLYWHEEL_SPINUP_MS = 500;        // Time to reach full speed (ms)

    // INTAKE
    private final double INTAKE_SPEED = 1;         // Intake IN power
    private final double OUTTAKE_SPEED = 0.6;          // Intake OUT power

    // APRILTAG ALIGNMENT
    private final double ALIGN_KP = 0.002;             // Turn aggressiveness
    private final double ALIGN_TOLERANCE = 5.0;        // Alignment tolerance (degrees)
    private final double ALIGN_MAX_SPEED = 0.3;        // Max turn speed during align
    private final double ALIGN_DEADBAND = 1.75;        // Stop adjusting below this error
    private final double ALIGN_ANGLE_OFFSET = 10.0;    // Aim offset (degrees)

    // ========================================================================
    // HARDWARE
    // ========================================================================
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intakeMotor, flyLeft, flyRight;

    Servo stage1Servo;   // Initial 170° push
    Servo stage2Servo;   // Mid 160° push
    Servo stage3Servo;   // Final feed to flywheels

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean isAligning = false;
    private AprilTagDetection targetTag = null;

    // Flywheel control
    private boolean flywheelsRunning = false;
    private long flywheelStartTime = 0;

    // Button press tracking for toggle
    private boolean lastRightBumper = false;

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

        // Initialize servos - UPDATE THESE NAMES TO MATCH YOUR HARDWARE CONFIG
        stage1Servo = hardwareMap.get(Servo.class, "stage1");  // 170° pusher
        stage2Servo = hardwareMap.get(Servo.class, "stage2");  // 160° pusher
        stage3Servo = hardwareMap.get(Servo.class, "stage3");  // Final feeder

        // Motor directions
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyRight.setDirection(DcMotor.Direction.REVERSE);
        flyLeft.setDirection(DcMotor.Direction.REVERSE);

        // Brake mode for drive
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize all servos to reset positions
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

        telemetry.addLine("=== MANUAL INDEPENDENT MODE ===");
        telemetry.addLine("Ready to test servos individually");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Display header
        telemetry.addLine("=== MANUAL INDEPENDENT CONTROL ===");
        telemetry.addLine("");

        // Handle all controls
        handleServoControls();
        handleFlywheelControl();
        handleIntakeControl();
        handleDriving();
        handleAprilTagAlign();

        // Display all status
        displayServoStatus();
        displaySystemStatus();

        telemetry.update();
    }

    // ========================================================================
    // SERVO CONTROLS - Each servo has dedicated buttons
    // ========================================================================
    private void handleServoControls() {
        telemetry.addLine("--- SERVO CONTROLS ---");

        // STAGE 1 CONTROL: D-pad Up/Down
        if (gamepad1.dpad_up) {
            stage1Servo.setPosition(STAGE1_PUSH_POS);
            telemetry.addLine(">>> STAGE 1: PUSHING");
        } else if (gamepad1.dpad_down) {
            stage1Servo.setPosition(STAGE1_RESET_POS);
            telemetry.addLine(">>> STAGE 1: RESET");
        }

        // STAGE 2 CONTROL: D-pad Left/Right
        if (gamepad1.dpad_left) {
            stage2Servo.setPosition(STAGE2_PUSH_POS);
            telemetry.addLine(">>> STAGE 2: PUSHING");
        } else if (gamepad1.dpad_right) {
            stage2Servo.setPosition(STAGE2_RESET_POS);
            telemetry.addLine(">>> STAGE 2: RESET");
        }

        // STAGE 3 CONTROL: Y/A buttons
        if (gamepad1.y) {
            stage3Servo.setPosition(STAGE3_PUSH_POS);
            telemetry.addLine(">>> STAGE 3: FEEDING");
        } else if (gamepad1.a) {
            stage3Servo.setPosition(STAGE3_RESET_POS);
            telemetry.addLine(">>> STAGE 3: RESET");
        }

        telemetry.addLine("D-pad Up/Down: Stage 1 | Left/Right: Stage 2 | Y/A: Stage 3");
        telemetry.addLine("");
    }

    // ========================================================================
    // FLYWHEEL CONTROL - Toggle on/off with right bumper
    // ========================================================================
    private void handleFlywheelControl() {
        // Toggle flywheels with right bumper
        if (gamepad1.right_bumper && !lastRightBumper) {
            flywheelsRunning = !flywheelsRunning;
            if (flywheelsRunning) {
                flywheelStartTime = System.currentTimeMillis();
            }
        }
        lastRightBumper = gamepad1.right_bumper;

        // Apply power with gradual spin-up
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
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(INTAKE_SPEED);
        } else if (gamepad1.x) {
            intakeMotor.setPower(-OUTTAKE_SPEED);
        } else {
            intakeMotor.setPower(0);
        }
    }

    // ========================================================================
    // DRIVING
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
        }

        // Drive or align
        if (isAligning) {
            autoAlign();
        } else {
            mecanum(forward, strafe, rotate);
        }
    }

    private void mecanum(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;
        double backLeftPower = forward - strafe + rotate;

        // Normalize powers
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
    // APRILTAG ALIGNMENT
    // ========================================================================
    private void handleAprilTagAlign() {
        // B button starts alignment
        if (gamepad1.b && !isAligning) {
            targetTag = findClosestAprilTag();
            if (targetTag != null) {
                isAligning = true;
            }
        }
    }

    private void autoAlign() {
        // Refresh target tag
        targetTag = findTagById(targetTag.id);

        // Lost sight of tag?
        if (targetTag == null) {
            isAligning = false;
            mecanum(0, 0, 0);
            return;
        }

        // Get bearing and apply offset
        double bearingDegrees = Math.toDegrees(targetTag.ftcPose.bearing) + ALIGN_ANGLE_OFFSET;

        // Check if aligned
        if (Math.abs(bearingDegrees) < ALIGN_TOLERANCE) {
            isAligning = false;
            mecanum(0, 0, 0);
            return;
        }

        // Stop if error is very small (prevents jitter)
        if (Math.abs(bearingDegrees) < ALIGN_DEADBAND) {
            mecanum(0, 0, 0);
            return;
        }

        // Calculate turn speed
        double turnSpeed = -bearingDegrees * ALIGN_KP;

        // Apply max speed limit
        if (Math.abs(turnSpeed) > ALIGN_MAX_SPEED) {
            turnSpeed = Math.signum(turnSpeed) * ALIGN_MAX_SPEED;
        }

        mecanum(0, 0, turnSpeed);
    }

    private AprilTagDetection findClosestAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return null;

        AprilTagDetection closest = null;
        double closestDist = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose.range < closestDist) {
                closestDist = detection.ftcPose.range;
                closest = detection;
            }
        }
        return closest;
    }

    private AprilTagDetection findTagById(int id) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == id) return detection;
        }
        return null;
    }

    // ========================================================================
    // TELEMETRY DISPLAY
    // ========================================================================
    private void displayServoStatus() {
        telemetry.addLine("--- SERVO POSITIONS ---");

        // Stage 1
        double stage1Pos = stage1Servo.getPosition();
        String stage1State = (stage1Pos > 0.5) ? "PUSH" : "RESET";
        telemetry.addData("Stage 1 (170°)", "%s | Pos: %.2f | Target: %.2f",
                stage1State, stage1Pos,
                (stage1Pos > 0.5) ? STAGE1_PUSH_POS : STAGE1_RESET_POS);

        // Stage 2
        double stage2Pos = stage2Servo.getPosition();
        String stage2State = (stage2Pos > 0.5) ? "PUSH" : "RESET";
        telemetry.addData("Stage 2 (160°)", "%s | Pos: %.2f | Target: %.2f",
                stage2State, stage2Pos,
                (stage2Pos > 0.5) ? STAGE2_PUSH_POS : STAGE2_RESET_POS);

        // Stage 3
        double stage3Pos = stage3Servo.getPosition();
        String stage3State = (stage3Pos > 0.4) ? "FEED" : "RESET";
        telemetry.addData("Stage 3 (Feed)", "%s | Pos: %.2f | Target: %.2f",
                stage3State, stage3Pos,
                (stage3Pos > 0.4) ? STAGE3_PUSH_POS : STAGE3_RESET_POS);

        telemetry.addLine("");
    }

    private void displaySystemStatus() {
        telemetry.addLine("--- SYSTEM STATUS ---");

        // Flywheels
        if (flywheelsRunning) {
            long elapsed = System.currentTimeMillis() - flywheelStartTime;
            double percent = Math.min(100.0, (double)elapsed / FLYWHEEL_SPINUP_MS * 100.0);
            telemetry.addData("Flywheels", "ON | %.0f%% | RB to toggle", percent);
        } else {
            telemetry.addData("Flywheels", "OFF | Right Bumper to start");
        }

        // Intake
        double intakePower = intakeMotor.getPower();
        if (Math.abs(intakePower) > 0.01) {
            String direction = (intakePower > 0) ? "IN" : "OUT";
            telemetry.addData("Intake", "%s | Power: %.2f", direction, Math.abs(intakePower));
        } else {
            telemetry.addData("Intake", "STOPPED | LB=IN | X=OUT");
        }

        // AprilTag
        if (isAligning) {
            telemetry.addData("AprilTag", "ALIGNING to Tag %d", targetTag.id);
        } else {
            telemetry.addData("AprilTag", "B to align");
        }

        telemetry.addLine("");
        telemetry.addLine("--- QUICK REFERENCE ---");
        telemetry.addLine("Stage 1: ↑↓ | Stage 2: ←→ | Stage 3: Y/A");
        telemetry.addLine("Flywheels: RB | Intake: LB/X | Align: B");
    }
}