package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;

public class BroccoliTeleop extends OpMode {
    CRServo left = null, right = null;

    @Override
    public void init() {
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");

        PwmControl leftP = (PwmControl) left;
        PwmControl rightP = (PwmControl) right;

        leftP.setPwmRange(new PwmControl.PwmRange(1000,2000));
        rightP.setPwmRange(new PwmControl.PwmRange(1000,2000));

        if(!leftP.isPwmEnabled()) {
            leftP.setPwmEnable();
        } if(!rightP.isPwmEnabled()) {
            rightP.setPwmEnable();
        }

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPower(0.5);
        right.setPower(0.5);
    }

    @Override
    public void loop() {
        double leftPower = 0, rightPower = 0;

        double turn = gamepad1.left_stick_x;
        double dp = gamepad1.right_trigger- gamepad1.left_trigger;
        double k = 1.5;

        if(gamepad1.right_bumper) {
            dp *= 0.4;
        }

        if(turn > 0) {
            leftPower = dp;
            rightPower = dp - k*turn*dp;
        } else {
            leftPower = dp + k*turn*dp;
            rightPower = dp;
        }

        // Constraints
        if(Math.abs(leftPower) > 1) {
            leftPower=Math.signum(leftPower);
        } if(Math.abs(rightPower) > 1) {
            rightPower=Math.signum(rightPower);
        }

        left.setPower(2*leftPower + 0.5);
        right.setPower(2*rightPower + 0.5);

    }
}
