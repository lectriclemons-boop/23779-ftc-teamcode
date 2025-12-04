/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Robot: Field Relative Mecanum Drive ZBrake", group = "Robot")
public class RobotTeleopMecanumFieldRelativeDriveZBrake extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left, back_left, front_right, back_right;

    IMU imu;

    // --- Z BRAKE / HEADING CONTROL ---
    double targetHeading = 0;
    boolean zBrakeActive = false;

    // --- RETURN TO ZERO ---
    boolean returnToZero = false;
    boolean prevB = false;

    // PID gain
    double kP = 0.01;

    @Override
    public void init() {

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_right = hardwareMap.get(DcMotor.class, "back_right");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {

        // Read inputs
        double rotateInput = gamepad1.right_stick_x;
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // --- STEP 3: B BUTTON EDGE DETECT → START RETURN-TO-ZERO ---
        if (gamepad1.b && !prevB) {
            returnToZero = true;
        }
        prevB = gamepad1.b;

        // Auto-stop once close enough to zero
        if (returnToZero && Math.abs(currentHeading) < Math.toRadians(2)) {
            returnToZero = false;
        }

        // --- Z-BRAKE ONLY IF NOT RETURNING TO ZERO ---
        if (!returnToZero) {
            if (Math.abs(rotateInput) > 0.05) {
                zBrakeActive = false;
                targetHeading = currentHeading;
            } else {
                if (!zBrakeActive) {
                    targetHeading = currentHeading;
                    zBrakeActive = true;
                }
            }
        }

        telemetry.addData("Heading(deg)", Math.toDegrees(currentHeading));
        telemetry.addData("ZBrake Active", zBrakeActive);
        telemetry.addData("Returning To Zero", returnToZero);
        telemetry.addData("Target Heading(deg)", Math.toDegrees(targetHeading));
        telemetry.update();

        // Reset yaw
        if (gamepad1.a) imu.resetYaw();

        // --- ROBOT-RELATIVE MODE (LB HELD) ---
        if (gamepad1.left_bumper) {
            drive(forward, right, rotateInput);
            return;
        }

        // --- TURN SELECTION PRIORITY ---
        double rotateFinal;

        // **1. RETURN HOME TO ZERO**
        if (returnToZero) {
            double error = AngleUnit.normalizeRadians(0 - currentHeading);
            rotateFinal = kP * error;
        }
        // **2. Z-BRAKE**
        else if (zBrakeActive) {
            double error = AngleUnit.normalizeRadians(targetHeading - currentHeading);
            rotateFinal = kP * error;
        }
        // **3. MANUAL INPUT**
        else {
            rotateFinal = rotateInput;
        }

        driveFieldRelative(forward, right, rotateFinal);
    }


    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }


    public void drive(double forward, double right, double rotate) {

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(br), Math.abs(bl)))));

        front_left.setPower(fl / max);
        front_right.setPower(fr / max);
        back_left.setPower(bl / max);
        back_right.setPower(br / max);
    }
}
