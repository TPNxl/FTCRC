package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FTCLib_test_hardware {
    MotorEx m0, m1, m2, m3;
    DcMotorEx dm0, dm1, dm2, dm3;

    MecanumDrive mec;

    RevIMU imu;

    public FTCLib_test_hardware() {}

    public void init(HardwareMap h) {
        // Motor setup
        m0 = new MotorEx(h, "m0");
        m1 = new MotorEx(h, "m1");
        m2 = new MotorEx(h, "m2");
        m3 = new MotorEx(h, "m3");

        m0.setRunMode(Motor.RunMode.VelocityControl);
        m1.setRunMode(Motor.RunMode.VelocityControl);
        m2.setRunMode(Motor.RunMode.VelocityControl);
        m3.setRunMode(Motor.RunMode.VelocityControl);

        m0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        dm0 = m0.motorEx;
        dm1 = m1.motorEx;
        dm2 = m2.motorEx;
        dm3 = m3.motorEx;

        dm0.setDirection(DcMotor.Direction.REVERSE);
        dm1.setDirection(DcMotor.Direction.FORWARD);
        dm2.setDirection(DcMotor.Direction.FORWARD);
        dm3.setDirection(DcMotor.Direction.REVERSE);

        // Drive setup
        mec = new MecanumDrive(m0, m1, m2, m3);

        // IMU setup
        imu = new RevIMU(h, "imu");
        imu.init();
    }
}
