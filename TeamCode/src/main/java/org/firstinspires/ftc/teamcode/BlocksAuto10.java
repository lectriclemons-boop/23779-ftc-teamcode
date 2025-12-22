package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlocksAuto10 (Blocks to Java)")
public class BlocksAuto10 extends LinearOpMode {

    private DcMotor back_left;
    private DcMotor back_right;
    private Servo stage1;
    private Servo stage2;
    private Servo stage3;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        stage1 = hardwareMap.get(Servo.class, "stage1");
        stage2 = hardwareMap.get(Servo.class, "stage2");
        stage3 = hardwareMap.get(Servo.class, "stage3");

        // Put initialization blocks here.
        Ball_to_shooting_position();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Back_up_to_shooting_position() {
        back_left.setPower(1);
        back_right.setPower(-1);
        sleep(600);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void Ball_to_shooting_position() {
        stage1.setPosition(0.4);
        sleep(1000);
        stage1.setPosition(1);
        sleep(1000);
        stage2.setPosition(0.83);
        sleep(1000);
        stage2.setPosition(0.3);
        sleep(1000);
        stage3.setPosition(0.25);
        sleep(1000);
        stage3.setPosition(0.5);
    }
}