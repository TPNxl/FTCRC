package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="WeirdOpMode")

public class FTCLib_weirdDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FTCLib_test_hardware x = new FTCLib_test_hardware();
        x.init(hardwareMap);

        waitForStart();

        ElapsedTime e = new ElapsedTime();

        x.mec.driveRobotCentric(0.7, 0.3, 0.15);

        e.reset();

        while(e.milliseconds() < 5000 || !isStopRequested()) {
            sleep(1);
        }

        x.mec.driveRobotCentric(0,0,0);
    }
}
