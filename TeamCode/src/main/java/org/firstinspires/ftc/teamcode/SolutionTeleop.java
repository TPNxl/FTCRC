package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group="Hardware", name="Solution")

public class SolutionTeleop extends OpMode {
    Hardware robot = new Hardware();

    double drive_speed = 0.4;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        if(Math.abs(left) > 0.9) {
            left = Math.signum(left);
        } if(Math.abs(right) > 0.9) {
            right = Math.signum(right);
        }


        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            drive_speed = 1;
        }
        else {
            drive_speed = 0.4;
        }
        left *= drive_speed;
        right *= drive_speed;

        robot.m0.setPower(left);
        robot.m1.setPower(right);
        robot.m2.setPower(left);
        robot.m3.setPower(right);
    }
}
