package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "hardware", name = "stupid")

public class CancerTeleop extends OpMode {

    Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double turn = Math.sin(gamepad1.left_stick_x*Math.PI/2);
        double throttle = gamepad1.right_trigger - gamepad1.left_trigger;

        double k = 1.4;

        if (turn < 0) {
            robot.m0.setPower(throttle + k*turn*throttle);
            robot.m1.setPower(throttle);
            if(gamepad1.b) {
                robot.m2.setPower(throttle);
                robot.m3.setPower(throttle);
            } else {
                robot.m2.setPower(throttle + k*turn*throttle);
                robot.m3.setPower(throttle);
            }

        } else {
            robot.m1.setPower(throttle - k*turn*throttle);
            robot.m0.setPower(throttle);
            if(gamepad1.b) {
                robot.m2.setPower(throttle);
                robot.m3.setPower(throttle);
            } else {
                robot.m2.setPower(throttle + k*turn*throttle);
                robot.m3.setPower(throttle);
            }
        }
    }
}
