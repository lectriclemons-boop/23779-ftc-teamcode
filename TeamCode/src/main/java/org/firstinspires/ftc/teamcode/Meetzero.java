package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Meet025 (Blocks to Java)")
@Disabled
public class Meetzero extends LinearOpMode {

    private DcMotor arm_motor;
    private DcMotor front_right;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor back_left;
    //private DcMotor extendo_arm;

    int current_extendo_value;
    int current_arm_value;
    double extendoASK_postion;
    float ASKposition;
    double DpadValueDown;
    double DpadValueUp;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        //extendo_arm = hardwareMap.get(DcMotor.class, "extendo_arm");

        // Put initialization blocks here.
        waitForStart();
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor.setTargetPosition(0);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                strafe_command(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }
    }

    private void strafe_command(float LSy, float LSx, float RSx) {
        float front_left_command;
        float front_right_command;
        float back_right_command;
        float back_left_command;

        front_left_command = (gamepad1.left_stick_y - gamepad1.right_stick_x) - gamepad1.left_stick_x;
        front_right_command = gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
        back_right_command = (gamepad1.left_stick_y + gamepad1.right_stick_x) - gamepad1.left_stick_x;
        back_left_command = (gamepad1.left_stick_y - gamepad1.right_stick_x) + gamepad1.left_stick_x;
    }
}