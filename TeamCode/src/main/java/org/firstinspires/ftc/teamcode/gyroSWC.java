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


@Autonomous(name="gyroSWC")

public class gyroSWC extends LinearOpMode {
    public Hardware robot = new Hardware();
    public BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait until we're told to go
        waitForStart();

        double Kp = 0.055;
        double Ki = 0;
        double Kd = 0.5;
        double Kt = 1;
        ElapsedTime e = new ElapsedTime();
        double correction = 0;
        double intErr = 0;
        double oldErr = 0;
        double origAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        do {
            // Wait for error to accumulate
            e.reset();
            sleep(20);

            // Gather data
            double error = -1 * (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - origAngle);
            double time = e.milliseconds();

            // Do math
            intErr += error * time;
            double dervErr = (error - oldErr) / time;

            oldErr = error;

            // Calculate correction
            double corr = Kt * (Kp * error + Ki * intErr + Kd * dervErr);

            robot.m0.setPower(0.5 - corr);
            robot.m1.setPower(0.5 + corr);
            robot.m2.setPower(0.5 - corr);
            robot.m3.setPower(0.5 + corr);

            telemetry.addData("angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();

        } while (!isStopRequested());
    }
}
