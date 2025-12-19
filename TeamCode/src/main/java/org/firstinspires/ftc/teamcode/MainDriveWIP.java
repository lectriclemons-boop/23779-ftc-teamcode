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
 * Simple robot-relative mecanum drive with AprilTag auto-align, Intake, and Track System
 *
 * Controls:
 * - Left stick: Forward/backward and strafe
 * - Right stick X: Rotation
 * - B button: Auto-align to nearest AprilTag
 * - A button: Intake IN
 * - X button: Intake OUT (reverse)
 * - Left Trigger: Hold for auto 3-shot burst with track system
 */
@TeleOp(name = "Main Drive WIP", group = "Robot")
public class MainDriveWIP extends OpMode {
    // Motors
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intakeMotor;
    DcMotor flyLeft;
    DcMotor flyRight;

    // Track system servos
    Servo funnelPusher;      // Pushes ball through funnel
    Servo rampStarter;       // Starts ball up the ramp
    Servo rampFinisher;      // Finishes ball up the ramp
    Servo feederServo;       // Feeds ball into shooter

    // ========================================================================
    // TUNING PARAMETERS - ADJUST THESE VALUES TO CUSTOMIZE ROBOT BEHAVIOR
    // ========================================================================

    // INTAKE TUNING
    private final double INTAKE_SPEED = 0.7;           // Intake speed 0-1 (try 0.6-1.0)
    private final double OUTTAKE_SPEED = 0.6;          // Reverse speed 0-1 (try 0.5-0.8)

    // TRACK SYSTEM TUNING - Sequential timing for ball movement
    // Stage 1: Funnel Pusher
    private final double FUNNEL_PUSH_POS = 0.7;        // Servo position when pushing (0-1)
    private final double FUNNEL_IDLE_POS = 0.2;        // Servo position when idle (0-1)
    private final int FUNNEL_PUSH_TIME_MS = 300;       // How long to push in ms (try 200-400)
    private final int FUNNEL_DELAY_MS = 0;             // Delay before starting (0 = immediate)

    // Stage 2: Ramp Starter (triggered after funnel completes)
    private final double RAMP_START_PUSH_POS = 0.8;    // Servo position when pushing (0-1)
    private final double RAMP_START_IDLE_POS = 0.1;    // Servo position when idle (0-1)
    private final int RAMP_START_PUSH_TIME_MS = 250;   // How long to push in ms (try 200-350)
    private final int RAMP_START_DELAY_MS = 100;       // Delay after funnel completes (try 50-200)

    // Stage 3: Ramp Finisher (triggered after ramp starter completes)
    private final double RAMP_FINISH_PUSH_POS = 0.75;  // Servo position when pushing (0-1)
    private final double RAMP_FINISH_IDLE_POS = 0.15;  // Servo position when idle (0-1)
    private final int RAMP_FINISH_PUSH_TIME_MS = 300;  // How long to push in ms (try 250-400)
    private final int RAMP_FINISH_DELAY_MS = 150;      // Delay after ramp starter completes (try 100-250)

    // SHOOTER TUNING
    private final double SHOOTER_SPEED = 0.70;          // Flywheel max speed 0-1 (try 0.8-1.0)
    private final int SHOOTER_SPINUP_TIME_MS = 1500;    // Time to reach full speed in ms (try 300-700)
    private final double FEEDER_PUSH_POS = 1.0;        // Feeder servo push position (0-1)
    private final double FEEDER_IDLE_POS = 0.0;        // Feeder servo idle position (0-1)
    private final int FEEDER_PUSH_TIME_MS = 250;       // How long to push ball in ms (try 200-400)

    // AUTO-CYCLE TUNING
    private final int CYCLE_DELAY_MS = 1000;           // Time between shots in ms (try 800-1500)
    private final int MAX_SHOTS = 3;                   // Max shots per trigger hold

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

    // Track system state tracking
    private enum TrackStage {
        IDLE,           // Not running
        FUNNEL,         // Funnel pusher active
        FUNNEL_WAIT,    // Waiting between funnel and ramp start
        RAMP_START,     // Ramp starter active
        RAMP_START_WAIT,// Waiting between ramp start and ramp finish
        RAMP_FINISH,    // Ramp finisher active
        COMPLETE        // Sequence complete
    }

    private TrackStage currentTrackStage = TrackStage.IDLE;
    private long trackStageStartTime = 0;

    // Shooter control
    private boolean shooterSpinning = false;           // Are flywheels spinning?
    private long shooterStartTime = 0;                 // When shooter started spinning
    private boolean shooterAtSpeed = false;            // Have flywheels reached target speed?
    private boolean feederActive = false;              // Is feeder pushing ball?
    private long feederStartTime = 0;                  // When feeder started
    private int shotCount = 0;                         // How many shots fired this cycle
    private long lastShotTime = 0;                     // When last shot completed

    @Override
    public void init() {
        // Get motors from hardware map
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        flyLeft = hardwareMap.get(DcMotor.class, "fly_left");
        flyRight = hardwareMap.get(DcMotor.class, "fly_right");

        // Get servos from hardware map
        funnelPusher = hardwareMap.get(Servo.class, "funnel_pusher");
        rampStarter = hardwareMap.get(Servo.class, "ramp_starter");
        rampFinisher = hardwareMap.get(Servo.class, "ramp_finisher");
        feederServo = hardwareMap.get(Servo.class, "feeder");

        // Reverse left motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse one flywheel motor so they spin in opposite directions
        flyRight.setDirection(DcMotor.Direction.REVERSE);

        // Set brake mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize all servos to idle positions
        funnelPusher.setPosition(FUNNEL_IDLE_POS);
        rampStarter.setPosition(RAMP_START_IDLE_POS);
        rampFinisher.setPosition(RAMP_FINISH_IDLE_POS);
        feederServo.setPosition(FEEDER_IDLE_POS);

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
        telemetry.addLine("Left Trigger: Hold for 3-shot burst");
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

        // Handle track system
        updateTrackSystem();

        // Handle shooter control
        controlShooter();

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

    // Update the sequential track system state machine
    private void updateTrackSystem() {
        long currentTime = System.currentTimeMillis();
        long elapsed = currentTime - trackStageStartTime;

        switch (currentTrackStage) {
            case IDLE:
                // Nothing to do
                break;

            case FUNNEL:
                // Funnel pusher is active
                if (elapsed >= FUNNEL_PUSH_TIME_MS) {
                    funnelPusher.setPosition(FUNNEL_IDLE_POS);
                    currentTrackStage = TrackStage.FUNNEL_WAIT;
                    trackStageStartTime = currentTime;
                }
                break;

            case FUNNEL_WAIT:
                // Waiting before starting ramp starter
                if (elapsed >= RAMP_START_DELAY_MS) {
                    rampStarter.setPosition(RAMP_START_PUSH_POS);
                    currentTrackStage = TrackStage.RAMP_START;
                    trackStageStartTime = currentTime;
                }
                break;

            case RAMP_START:
                // Ramp starter is active
                if (elapsed >= RAMP_START_PUSH_TIME_MS) {
                    rampStarter.setPosition(RAMP_START_IDLE_POS);
                    currentTrackStage = TrackStage.RAMP_START_WAIT;
                    trackStageStartTime = currentTime;
                }
                break;

            case RAMP_START_WAIT:
                // Waiting before starting ramp finisher
                if (elapsed >= RAMP_FINISH_DELAY_MS) {
                    rampFinisher.setPosition(RAMP_FINISH_PUSH_POS);
                    currentTrackStage = TrackStage.RAMP_FINISH;
                    trackStageStartTime = currentTime;
                }
                break;

            case RAMP_FINISH:
                // Ramp finisher is active
                if (elapsed >= RAMP_FINISH_PUSH_TIME_MS) {
                    rampFinisher.setPosition(RAMP_FINISH_IDLE_POS);
                    currentTrackStage = TrackStage.COMPLETE;
                    trackStageStartTime = currentTime;
                }
                break;

            case COMPLETE:
                // Track sequence is complete, ready for next cycle
                currentTrackStage = TrackStage.IDLE;
                break;
        }

        // Display track system status
        if (currentTrackStage != TrackStage.IDLE) {
            telemetry.addData("Track Stage", currentTrackStage.toString());
        }
    }

    // Start the track system sequence
    private void startTrackSequence() {
        if (currentTrackStage == TrackStage.IDLE || currentTrackStage == TrackStage.COMPLETE) {
            currentTrackStage = TrackStage.FUNNEL;
            trackStageStartTime = System.currentTimeMillis();
            funnelPusher.setPosition(FUNNEL_PUSH_POS);
        }
    }

    // Control shooter with left trigger - auto-cycles 3 shots
    private void controlShooter() {
        float triggerValue = gamepad1.left_trigger;
        long currentTime = System.currentTimeMillis();

        // Start shooting sequence when trigger pressed
        if (triggerValue > 0.1 && !shooterSpinning) {
            shooterSpinning = true;
            shooterStartTime = currentTime;
            shooterAtSpeed = false;
            shotCount = 0;
            startTrackSequence();  // Start the first ball through track
        }

        // Stop everything if trigger released
        if (triggerValue < 0.1 && shooterSpinning) {
            shooterSpinning = false;
            shooterAtSpeed = false;
            feederActive = false;
            shotCount = 0;
            flyLeft.setPower(0);
            flyRight.setPower(0);
            feederServo.setPosition(FEEDER_IDLE_POS);
            currentTrackStage = TrackStage.IDLE;
            // Reset all track servos
            funnelPusher.setPosition(FUNNEL_IDLE_POS);
            rampStarter.setPosition(RAMP_START_IDLE_POS);
            rampFinisher.setPosition(RAMP_FINISH_IDLE_POS);
        }

        // Manage shooting sequence
        if (shooterSpinning) {
            long elapsed = currentTime - shooterStartTime;

            // Spin flywheels
            flyLeft.setPower(SHOOTER_SPEED);
            flyRight.setPower(SHOOTER_SPEED);

            // Wait for spin-up on first shot
            if (!shooterAtSpeed && elapsed >= SHOOTER_SPINUP_TIME_MS) {
                shooterAtSpeed = true;
                lastShotTime = currentTime;
            }

            // Fire shots if at speed and haven't reached max
            if (shooterAtSpeed && shotCount < MAX_SHOTS) {
                long timeSinceLastShot = currentTime - lastShotTime;

                // Start feeding
                if (!feederActive && timeSinceLastShot >= CYCLE_DELAY_MS) {
                    feederActive = true;
                    feederStartTime = currentTime;
                    feederServo.setPosition(FEEDER_PUSH_POS);
                }

                // Finish feeding
                if (feederActive && (currentTime - feederStartTime) >= FEEDER_PUSH_TIME_MS) {
                    feederServo.setPosition(FEEDER_IDLE_POS);
                    feederActive = false;
                    shotCount++;
                    lastShotTime = currentTime;

                    // Start next ball through track if not done
                    if (shotCount < MAX_SHOTS) {
                        startTrackSequence();
                    }
                }
            }

            telemetry.addData("Shooter", "ACTIVE");
            telemetry.addData("Shots Fired", "%d/%d", shotCount, MAX_SHOTS);
            telemetry.addData("Feeder", feederActive ? "FEEDING" : "READY");
        } else {
            telemetry.addData("Shooter", "OFF");
        }
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