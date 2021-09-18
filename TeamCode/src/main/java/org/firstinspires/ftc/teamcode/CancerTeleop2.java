package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group="Hardware", name ="stupid2")

public class CancerTeleop2 extends OpMode {

    Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;

        robot.m0.setPower(left);
        robot.m1.setPower(right);
        robot.m2.setPower(left);
        robot.m3.setPower(right);
    }
}
