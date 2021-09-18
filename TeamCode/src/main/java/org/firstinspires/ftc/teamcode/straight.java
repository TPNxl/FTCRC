package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="straight")

public class straight extends LinearOpMode {
    public Hardware robot = new Hardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while(!isStopRequested()) {
            robot.m0.setPower(0.2);
            robot.m1.setPower(0.2);
            robot.m2.setPower(0.2);
            robot.m3.setPower(0.2);
        }
    }
}
