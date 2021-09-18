package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="PokemonTheAnime")

public class PokemonTheAnime extends OpMode {
    public Hardware robot = new Hardware();
    public BNO055IMU imu;

    Orientation original;

    @Override
    public void init() {
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

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        original = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    @Override
    public void loop() {
        // Set power
        robot.m0.setPower(0.5);
        robot.m1.setPower(0.6);
        robot.m2.setPower(0.5);
        robot.m3.setPower(0.6);

        Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Dr", o.firstAngle - original.firstAngle);
        telemetry.addData("Dp", o.secondAngle - original.secondAngle);
        telemetry.addData("Dt", o.thirdAngle - original.thirdAngle);
        telemetry.addData("gravity", imu.getGravity().toString());
        Acceleration a = imu.getAcceleration();
        telemetry.addData("d^2z", a.zAccel);
        telemetry.addData("d^2y", a.yAccel);
        telemetry.addData("d^2x", a.xAccel);
        Position p = imu.getPosition();
        telemetry.addData("z", p.z);
        telemetry.addData("y", p.y);
        telemetry.addData("x", p.x);
        telemetry.update();
    }
}
