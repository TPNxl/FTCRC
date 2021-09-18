package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    DcMotor m0, m1, m2, m3;

    public Robot() {
    }

    public void init(HardwareMap hwMap) {
        m0 = hwMap.get(DcMotor.class, "m0");
        m1 = hwMap.get(DcMotor.class, "m1");
        m2 = hwMap.get(DcMotor.class, "m2");
        m3 = hwMap.get(DcMotor.class, "m3");

        m0.setPower(0);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);

        m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m0.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m1.setDirection(DcMotorSimple.Direction.FORWARD);
        m3.setDirection(DcMotorSimple.Direction.FORWARD);

        m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
