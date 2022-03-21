package org.firstinspires.ftc.teamcode.hardwarewrap;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// wrapper class for gyro sensor
public class GyroWrap {

    // telemetry debugging device
    public Telemetry tele;

    // gyro sensor reference
    public RevIMU gyro;

    // device name
    public String name;

    // initialize device
    public GyroWrap(Telemetry tele, HardwareMap map, String name) {

        // telemetry debugging device
        this.tele = tele;

        // debugging data
        tele.addData("init gyro start", name);
        tele.update();

        // get gyro reference
        gyro = new RevIMU(map, name);
        gyro.init();
        this.name = name;

        // debugging data
        tele.addData("init gyro complete", name);
        tele.update();
    }

    // terminate device
    public void stop() {
        gyro.disable();
    }

    // get current angle
    public double getHeading() {
        return gyro.getHeading();
    }

    // reset origin angle to current orientation
    public void setOrigin() {
        gyro.reset();
    }
}
