package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="F0teleop")

public class F0teleop extends OpMode {
    F0 f0 = new F0();

    @Override
    public void init() {
        try {
            f0.init(hardwareMap);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
        double drive = gamepad1.right_trigger - 0.4*gamepad1.left_trigger;
        double turn = 0.2*gamepad1.left_stick_x;

        f0.drive.arcadeDrive(drive, turn);
    }
}
