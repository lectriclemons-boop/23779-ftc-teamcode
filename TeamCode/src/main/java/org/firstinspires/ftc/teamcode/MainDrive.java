package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * Simple robot-relative mecanum drive with AprilTag auto-align, Intake, and Turret
 *
 * Controls:
 * - Left stick: Forward/backward and strafe
 * - Right stick X: Rotation
 * - B button: Auto-align to nearest AprilTag
 * - A button: Intake IN
 * - X button: Intake OUT (reverse)
 * - D-Pad Up: Turret Position 1
 * - D-Pad Right: Turret Position 2
 * - D-Pad Down: Turret Position 3
 */
@TeleOp(name = "Main", group = "Robot")
public class MainDrive extends OpMode {
    // Motors
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intakeMotor;
    DcMotor sorterDrive;

    // ========================================================================
    // TUNING PARAMETERS - ADJUST THESE VALUES TO CUSTOMIZE ROBOT BEHAVIOR
    // ========================================================================

    // INTAKE TUNING
    private final double INTAKE_SPEED = 0.7;           // Intake speed 0-1 (try 0.6-1.0)
    private final double OUTTAKE_SPEED = 0.6;          // Reverse speed 0-1 (try 0.5-0.8)

    // TURRET TUNING
    private final double TURRET_SPEED = 0.4;           // Turret rotation speed 0-1 (try 0.3-0.6)
    private final double TURRET_POS1_REVS = 0.0;       // Position 1 in motor revolutions (starting position)
    private final double TURRET_POS2_REVS = 1.5;       // Position 2 in motor revolutions (try 1.0-3.0)
    private final double TURRET_POS3_REVS = 3.0;       // Position 3 in motor revolutions (try 2.0-5.0)
    private final double TURRET_TOLERANCE = 0.05;      // How close is "close enough" in revs (try 0.02-0.1)

    // APRILTAG ALIGNMENT TUNING
    private final double ALIGN_KP = 0.002;             // Turn aggressiveness (try 0.02-0.05)
    private final double ALIGN_TOLERANCE = 5.0;        // "Close enough" in degrees (try 1.0-3.0)
    private final double ALIGN_MAX_SPEED = 0.1;        // Max turn speed 0-1 (try 0.3-0.5)
    private final double ALIGN_DEADBAND = 1.75;        // Stop adjusting below this error (degrees)
    private final double ALIGN_ANGLE_OFFSET = 10.0;    // Add degrees to aim left(-) or right(+)

    // SMOOTH BRAKING TUNING
    private final double BRAKE_TIME_MS = 100;          // Brake time in milliseconds
    // 50-80 = aggressive (quick stop)
    // 100-150 = moderate (balanced)
    // 150-200 = gentle (very smooth)

    // ========================================================================
    // END OF TUNING PARAMETERS
    // ========================================================================

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean isAligning = false;
    private AprilTagDetection targetTag = null;

    // Smooth braking
    private boolean isBraking = false;                 // Are we currently braking?
    private double[] lastPowers = new double[4];       // Store last motor powers
    private long brakeStartTime = 0;                   // When we started braking

    // Turret control
    private int targetTurretPosition = 1;              // Which position we're moving to (1, 2, or 3)
    private boolean turretMoving = false;              // Is turret currently moving?
    private boolean dpadUpPressed = false;             // Button state tracking
    private boolean dpadRightPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void init() {
        // Get motors from hardware map
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        sorterDrive = hardwareMap.get(DcMotor.class, "sorter_drive");

        // Reverse left motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup sorter motor with encoder
        sorterDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize AprilTag detection
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new android.util.Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();
    }

    @Override
    public void loop() {
        // Display controls
        telemetry.addLine("Left stick: Drive");
        telemetry.addLine("Right stick X: Turn");
        telemetry.addLine("B: Auto-align to AprilTag");
        telemetry.addLine("A: Intake IN | X: Intake OUT");
        telemetry.addLine("D-Pad Up/Right/Down: Turret Positions 1/2/3");
        telemetry.addLine("");
        telemetry.addLine("Auto-align: " + (isAligning ? "ACTIVE" : "INACTIVE"));

        // Show AprilTag info
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("AprilTags seen", detections.size());
        for (AprilTagDetection detection : detections) {
            telemetry.addData("Tag ID", detection.id);
            telemetry.addData("  Bearing", "%.1f degrees", Math.toDegrees(detection.ftcPose.bearing));
        }

        // Handle intake controls
        controlIntake();

        // Handle turret controls
        controlTurret();

        // Get joystick inputs
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Check if driver is moving the sticks (use small deadzone of 0.05)
        boolean driverIsMoving = Math.abs(forward) > 0.05 ||
                Math.abs(strafe) > 0.05 ||
                Math.abs(rotate) > 0.05;

        // B button starts auto-align
        if (gamepad1.b && !isAligning) {
            targetTag = findClosestAprilTag();
            if (targetTag != null) {
                isAligning = true;
                telemetry.addLine("Starting alignment...");
            } else {
                telemetry.addLine("No AprilTag found!");
            }
        }

        // Cancel auto-align if driver moves
        if (driverIsMoving && isAligning) {
            isAligning = false;
            telemetry.addLine("Alignment cancelled");
        }

        // === SMOOTH BRAKING LOGIC ===
        // If driver was moving but now stopped, start smooth braking
        if (!driverIsMoving && !isAligning && !isBraking) {
            // Check if motors are actually moving (not already stopped)
            boolean motorsMoving = Math.abs(frontLeftDrive.getPower()) > 0.01;
            if (motorsMoving) {
                startBraking();  // Begin the smooth brake sequence
            }
        }

        // If driver starts moving again, cancel braking immediately
        if (driverIsMoving && isBraking) {
            isBraking = false;
        }

        // Execute the appropriate mode
        if (isAligning) {
            autoAlign();
        } else if (isBraking) {
            executeBraking();  // Smoothly bring motors to stop
        } else {
            drive(forward, strafe, rotate);
        }

        telemetry.update();
    }

    // Control intake based on buttons
    private void controlIntake() {
        if (gamepad1.a) {
            // Intake IN
            intakeMotor.setPower(INTAKE_SPEED);
            telemetry.addData("Intake", "IN (%.2f)", INTAKE_SPEED);
        } else if (gamepad1.x) {
            // Intake OUT (reverse)
            intakeMotor.setPower(-OUTTAKE_SPEED);
            telemetry.addData("Intake", "OUT (%.2f)", OUTTAKE_SPEED);
        } else {
            // Stop intake
            intakeMotor.setPower(0);
            telemetry.addData("Intake", "STOPPED");
        }
    }

    // Control turret position with D-Pad
    private void controlTurret() {
        // Button press detection (only trigger once per press)
        if (gamepad1.dpad_up && !dpadUpPressed) {
            dpadUpPressed = true;
            setTurretTarget(1);
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_right && !dpadRightPressed) {
            dpadRightPressed = true;
            setTurretTarget(2);
        } else if (!gamepad1.dpad_right) {
            dpadRightPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            dpadDownPressed = true;
            setTurretTarget(3);
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // Move turret to target position if needed
        if (turretMoving) {
            updateTurretMovement();
        }

        // Telemetry
        double currentRevs = sorterDrive.getCurrentPosition() / getTicksPerRev(sorterDrive);
        telemetry.addData("Turret Position", targetTurretPosition);
        telemetry.addData("Turret Revs", "%.2f", currentRevs);
        telemetry.addData("Turret Status", turretMoving ? "MOVING" : "STOPPED");
    }

    // Set new turret target position
    private void setTurretTarget(int position) {
        targetTurretPosition = position;
        turretMoving = true;
    }

    // Move turret toward target position
    private void updateTurretMovement() {
        // Get target position in revolutions
        double targetRevs;
        switch (targetTurretPosition) {
            case 1:
                targetRevs = TURRET_POS1_REVS;
                break;
            case 2:
                targetRevs = TURRET_POS2_REVS;
                break;
            case 3:
                targetRevs = TURRET_POS3_REVS;
                break;
            default:
                targetRevs = TURRET_POS1_REVS;
        }

        // Get current position in revolutions
        double currentRevs = sorterDrive.getCurrentPosition() / getTicksPerRev(sorterDrive);

        // Calculate error
        double error = targetRevs - currentRevs;

        // Check if we've reached the target
        if (Math.abs(error) < TURRET_TOLERANCE) {
            sorterDrive.setPower(0);
            turretMoving = false;
            return;
        }

        // Move toward target
        if (error > 0) {
            sorterDrive.setPower(TURRET_SPEED);  // Move forward
        } else {
            sorterDrive.setPower(-TURRET_SPEED); // Move backward
        }
    }

    // Get ticks per revolution for a motor (common FTC motors)
    private double getTicksPerRev(DcMotor motor) {
        // GoBILDA 5203 series 435 RPM motor: 537.7 ticks/rev
        return 537.7;
    }

    // Start the smooth braking sequence
    private void startBraking() {
        isBraking = true;
        brakeStartTime = System.currentTimeMillis();

        // Remember current motor powers so we can ramp them down
        lastPowers[0] = frontLeftDrive.getPower();
        lastPowers[1] = frontRightDrive.getPower();
        lastPowers[2] = backLeftDrive.getPower();
        lastPowers[3] = backRightDrive.getPower();
    }

    // Gradually reduce motor power to zero
    private void executeBraking() {
        // How long have we been braking?
        long elapsed = System.currentTimeMillis() - brakeStartTime;

        // Are we done braking?
        if (elapsed >= BRAKE_TIME_MS) {
            // Stop all motors and exit braking mode
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            isBraking = false;
            return;
        }

        // Calculate how much to reduce power (1.0 at start -> 0.0 at end)
        double factor = 1.0 - (elapsed / BRAKE_TIME_MS);

        // Apply the reduced power to each motor
        frontLeftDrive.setPower(lastPowers[0] * factor);
        frontRightDrive.setPower(lastPowers[1] * factor);
        backRightDrive.setPower(lastPowers[2] * factor);
        backLeftDrive.setPower(lastPowers[3] * factor);
    }

    // Basic mecanum drive
    private void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;
        double backLeftPower = forward - strafe + rotate;

        // Find max power
        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // Scale to keep all powers under 1.0
        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }

    // Auto-align to AprilTag
    private void autoAlign() {
        // Refresh target tag
        targetTag = findTagById(targetTag.id);

        // Lost sight of tag?
        if (targetTag == null) {
            telemetry.addLine("Lost AprilTag!");
            isAligning = false;
            drive(0, 0, 0);
            return;
        }

        // Get bearing (angle to tag) and apply offset
        double bearingDegrees = Math.toDegrees(targetTag.ftcPose.bearing) + ALIGN_ANGLE_OFFSET;

        // Check if aligned
        if (Math.abs(bearingDegrees) < ALIGN_TOLERANCE) {
            telemetry.addLine("ALIGNED!");
            isAligning = false;
            drive(0, 0, 0);
            return;
        }

        // If error is very small, don't move at all (prevents jitter)
        if (Math.abs(bearingDegrees) < ALIGN_DEADBAND) {
            drive(0, 0, 0);
            telemetry.addData("Fine-tuning", "holding position");
            return;
        }

        // Calculate turn speed (negative to fix direction)
        double turnSpeed = -bearingDegrees * ALIGN_KP;

        // Apply max speed limit only (no minimum speed for smooth approach)
        if (Math.abs(turnSpeed) > ALIGN_MAX_SPEED) {
            turnSpeed = Math.signum(turnSpeed) * ALIGN_MAX_SPEED;
        }

        // Turn only (no forward/strafe)
        drive(0, 0, turnSpeed);

        telemetry.addData("Aligning to Tag", targetTag.id);
        telemetry.addData("Bearing error", "%.1f deg", bearingDegrees);
        telemetry.addData("Turn speed", "%.2f", turnSpeed);
    }

    // Find closest AprilTag
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

    // Find tag by ID
    private AprilTagDetection findTagById(int id) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == id) return detection;
        }
        return null;
    }
}