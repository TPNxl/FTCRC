// Hardware class for F0

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class F0 {
    RevIMU imu;
    CRServo l, r;
    ServoControllerEx scl, scr;
    DifferentialDrive drive;
    VoltageSensor v;

    public F0() {}

    public void init(HardwareMap h) throws Exception {
        imu = new RevIMU(h, "imu");

        l = h.get(CRServo.class, "sl");
        r = h.get(CRServo.class, "sr");

        v = h.get(VoltageSensor.class, "v");

        scl = (ServoControllerEx)l.getController();
        scr = (ServoControllerEx)r.getController();

        l.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setDirection(DcMotorSimple.Direction.FORWARD);

        scl.setServoPwmRange(0, new PwmControl.PwmRange(1000,2000));
        scr.setServoPwmRange(0, new PwmControl.PwmRange(1000,2000));

        scl.pwmEnable();
        scr.pwmEnable();

        l.setPower(0);
        r.setPower(0);

        if(Math.abs(v.getVoltage() - 12) > 0.1) {
            throw new Exception("Buck boost not working correctly");
        }

        drive = new DifferentialDrive((Motor)l,(Motor)r);
    }

}
