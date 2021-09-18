package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="PcontrollerProblem")

public class PcontrollerProblem extends LinearOpMode {
    public Hardware robot = new Hardware();
    public BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait until we're told to go
        waitForStart();

        ElapsedTime e = new ElapsedTime();
        double origAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // The starting angle of the robot

        // Repeat until the stop button is pressed
        do {
            // Wait for error to accumulate (using ElapsedTime)
            e.reset();
            sleep(20);

            // Gather data
            double newAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double time = e.milliseconds();


            telemetry.addData("angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle); // For telemetry
            telemetry.update();

            /* Your code goes here. Your code should look something like this:
            1. Calculate the error
            2. Calculate the derivative of the error using the time variable
            3. Multiply each by a variable coefficient to get a proportional and derivative correction factor
            4. Sum the correction factors
            5. Change the motor powers based on the total correction factor.
             */

        } while (!isStopRequested());
    }
}
