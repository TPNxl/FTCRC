package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="broccoli")

public class BroccoliTest extends LinearOpMode {
    CRServo sm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup

        sm = hardwareMap.get(CRServo.class, "sm");
        PwmControl smc = (PwmControl)sm;
        smc.setPwmRange(new PwmControl.PwmRange(1000,2000));

        if(!smc.isPwmEnabled()) {
            smc.setPwmEnable();
        }

        // Test
        ElapsedTime e = new ElapsedTime();

        sm.setPower(0.4);

        e.reset();

        while(e.milliseconds() < 1000) {

        }

        sm.setPower(0.6);

        e.reset();

        while(e.milliseconds() < 1000) {

        }

        sm.setPower(0.5);
    }
}
