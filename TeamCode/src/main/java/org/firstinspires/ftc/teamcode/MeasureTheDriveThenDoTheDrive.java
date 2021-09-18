package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="MeasureThenDrive")

public class MeasureTheDriveThenDoTheDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FTCLib_test_hardware x = new FTCLib_test_hardware();
        x.init(hardwareMap);

        waitForStart();
        /*

        x.imu.startAccelerationIntegration(new Position(), new Velocity(), 200);

        Position orig = x.imu.getPosition();

        while(!isStopRequested()) {
            Position p = x.imu.getPosition();

            if(p.z - orig.z > 5) {
                break;
            } else {
                x.mec.driveRobotCentric(0,0.2,0);
                telemetry.addData("z", p.z);
                telemetry.addData("y", p.y);
                telemetry.addData("x", p.x);
                telemetry.update();
            }

        }

         */
    }
}
