package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="LibPID")

public class LibPID extends LinearOpMode {
    FTCLib_test_hardware x = new FTCLib_test_hardware();
    @Override
    public void runOpMode() throws InterruptedException {
        x.init(hardwareMap);

        waitForStart();

        PIDFController pidf = new PIDFController(0.15,0,0,0);

        double origAngle = x.imu.getAngles()[2];

        pidf.setSetPoint(0);

        while(!isStopRequested()) {
            double angle = x.imu.getAngles()[2];
            double error = angle - origAngle;
            telemetry.addData("orig", origAngle);
            telemetry.addData("angle", angle);
            telemetry.addData("error", error);
            double calc = pidf.calculate(error);
            telemetry.update();
            x.mec.driveRobotCentric(0.5, 0, -calc);
        }

    }
}
