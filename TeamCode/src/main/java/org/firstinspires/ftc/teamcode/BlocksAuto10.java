package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlocksAuto10 (Blocks to Java)")
public class BlocksAuto10 extends LinearOpMode {

    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor fly_right;
    private DcMotor fly_left;
    private Servo stage1;
    private Servo stage2;
    private Servo stage3;

    double Stage1__start;
    int Stage1_End;
    double Stage2_End;
    double Stage2_Start;
    double Stage3_End;
    double Stage3_Start;

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
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        fly_right = hardwareMap.get(DcMotor.class, "fly_right");
        fly_left = hardwareMap.get(DcMotor.class, "fly_left");
        stage1 = hardwareMap.get(Servo.class, "stage1");
        stage2 = hardwareMap.get(Servo.class, "stage2");
        stage3 = hardwareMap.get(Servo.class, "stage3");

        // Put initialization blocks here.
        Stage1__start = 0.4;
        Stage1_End = 1;
        Stage2_Start = 0.83;
        Stage2_End = 0.3;
        Stage3_Start = 0.25;
        Stage3_End = 0.5;
        waitForStart();
        Back_up_to_shooting_position();
        start_Fly_wheels();
        Ball_to_shooting_position_and_shoot2();
        Ball_to_shooting_position_and_shoot2();
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
        front_left.setPower(1);
        front_right.setPower(-1);
        sleep(600);
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void start_Fly_wheels() {
        fly_right.setPower(-0.6);
        fly_left.setPower(0.6);
    }

    /**
     * Describe this function...
     */
    private void Ball_to_shooting_position_and_shoot() {
        stage1.setPosition(Stage1__start);
        sleep(500);
        stage1.setPosition(Stage1_End);
        sleep(500);
        stage2.setPosition(Stage2_Start);
        sleep(500);
        stage2.setPosition(Stage2_End);
        sleep(500);
        stage1.setPosition(Stage1__start);
        sleep(500);
        stage3.setPosition(Stage3_Start);
        sleep(500);
        stage3.setPosition(Stage3_End);
        sleep(500);
        stage1.setPosition(Stage1__start);
        stage2.setPosition(Stage2_Start);
        stage3.setPosition(Stage3_Start);
    }

    /**
     * Describe this function...
     */
    private void Ball_to_shooting_position_and_shoot2() {
        stage1.setPosition(Stage1_End);
        stage2.setPosition(Stage2_End);
        stage3.setPosition(Stage3_End);
        stage1.setPosition(Stage1__start);
        sleep(500);
        stage2.setPosition(Stage2_Start);
        sleep(500);
        stage3.setPosition(Stage2_Start);
        sleep(500);
        stage1.setPosition(Stage1_End);
        stage2.setPosition(Stage2_End);
        stage3.setPosition(Stage3_End);
    }
}