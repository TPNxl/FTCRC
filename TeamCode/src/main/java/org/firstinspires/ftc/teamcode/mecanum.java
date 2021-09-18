package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="fuckingstupid2")

public class mecanum extends OpMode {
    DcMotor m0, m1, m2, m3;

    @Override
    public void init() {
        m0 = hardwareMap.get(DcMotor.class,"m0");
        m1 = hardwareMap.get(DcMotor.class,"m1");
        m2 = hardwareMap.get(DcMotor.class,"m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");

        m0.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

    }
}
