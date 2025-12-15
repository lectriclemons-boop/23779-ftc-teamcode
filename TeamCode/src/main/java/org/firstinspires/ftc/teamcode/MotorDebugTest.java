package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Simple motor test OpMode to diagnose motor issues
 * Tests each motor individually and shows their status
 */
@TeleOp(name = "Motor Debug Test", group = "Debug")
public class MotorDebugTest extends OpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    @Override
    public void init() {
        telemetry.addLine("Initializing motors...");
        telemetry.update();

        try {
            frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
            telemetry.addLine("✓ Front Left found");
        } catch (Exception e) {
            telemetry.addLine("✗ Front Left NOT FOUND: " + e.getMessage());
        }

        try {
            backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
            telemetry.addLine("✓ Back Left found");
        } catch (Exception e) {
            telemetry.addLine("✗ Back Left NOT FOUND: " + e.getMessage());
        }

        try {
            frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
            telemetry.addLine("✓ Front Right found");
        } catch (Exception e) {
            telemetry.addLine("✗ Front Right NOT FOUND: " + e.getMessage());
        }

        try {
            backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
            telemetry.addLine("✓ Back Right found");
        } catch (Exception e) {
            telemetry.addLine("✗ Back Right NOT FOUND: " + e.getMessage());
        }

        // Set all motors to default forward direction (no reversal)
        if (frontLeftDrive != null) {
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (frontRightDrive != null) {
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (backLeftDrive != null) {
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (backRightDrive != null) {
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("\n=== READY TO TEST ===");
        telemetry.addLine("D-pad Up: Test Front Left");
        telemetry.addLine("D-pad Down: Test Back Left");
        telemetry.addLine("D-pad Right: Test Front Right");
        telemetry.addLine("D-pad Left: Test Back Right");
        telemetry.addLine("Y: Test ALL motors");
        telemetry.addLine("X: Stop all motors");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Stop all motors first
        if (frontLeftDrive != null) frontLeftDrive.setPower(0);
        if (frontRightDrive != null) frontRightDrive.setPower(0);
        if (backLeftDrive != null) backLeftDrive.setPower(0);
        if (backRightDrive != null) backRightDrive.setPower(0);

        // Test individual motors
        if (gamepad1.dpad_up && frontLeftDrive != null) {
            frontLeftDrive.setPower(0.5);
            telemetry.addLine(">>> TESTING FRONT LEFT at 50% power");
            telemetry.addData("Front Left Current", frontLeftDrive.getCurrentPosition());
        }

        if (gamepad1.dpad_down && backLeftDrive != null) {
            backLeftDrive.setPower(0.5);
            telemetry.addLine(">>> TESTING BACK LEFT at 50% power");
            telemetry.addData("Back Left Current", backLeftDrive.getCurrentPosition());
        }

        if (gamepad1.dpad_right && frontRightDrive != null) {
            frontRightDrive.setPower(0.5);
            telemetry.addLine(">>> TESTING FRONT RIGHT at 50% power");
            telemetry.addData("Front Right Current", frontRightDrive.getCurrentPosition());
        }

        if (gamepad1.dpad_left && backRightDrive != null) {
            backRightDrive.setPower(0.5);
            telemetry.addLine(">>> TESTING BACK RIGHT at 50% power");
            telemetry.addData("Back Right Current", backRightDrive.getCurrentPosition());
        }

        // Test all motors at once
        if (gamepad1.y) {
            if (frontLeftDrive != null) frontLeftDrive.setPower(0.5);
            if (frontRightDrive != null) frontRightDrive.setPower(0.5);
            if (backLeftDrive != null) backLeftDrive.setPower(0.5);
            if (backRightDrive != null) backRightDrive.setPower(0.5);
            telemetry.addLine(">>> TESTING ALL MOTORS at 50% power");
        }

        // Motor status
        telemetry.addLine("\n=== MOTOR STATUS ===");

        if (frontLeftDrive != null) {
            telemetry.addData("Front Left", "Power: %.2f, Pos: %d",
                    frontLeftDrive.getPower(), frontLeftDrive.getCurrentPosition());
        } else {
            telemetry.addLine("Front Left: NOT CONNECTED");
        }

        if (backLeftDrive != null) {
            telemetry.addData("Back Left", "Power: %.2f, Pos: %d",
                    backLeftDrive.getPower(), backLeftDrive.getCurrentPosition());
        } else {
            telemetry.addLine("Back Left: NOT CONNECTED");
        }

        if (frontRightDrive != null) {
            telemetry.addData("Front Right", "Power: %.2f, Pos: %d",
                    frontRightDrive.getPower(), frontRightDrive.getCurrentPosition());
        } else {
            telemetry.addLine("Front Right: NOT CONNECTED");
        }

        if (backRightDrive != null) {
            telemetry.addData("Back Right", "Power: %.2f, Pos: %d",
                    backRightDrive.getPower(), backRightDrive.getCurrentPosition());
        } else {
            telemetry.addLine("Back Right: NOT CONNECTED");
        }

        telemetry.addLine("\n=== CONTROLS ===");
        telemetry.addLine("D-pad Up: Front Left");
        telemetry.addLine("D-pad Down: Back Left");
        telemetry.addLine("D-pad Right: Front Right");
        telemetry.addLine("D-pad Left: Back Right");
        telemetry.addLine("Y: ALL motors");
        telemetry.update();
    }
}