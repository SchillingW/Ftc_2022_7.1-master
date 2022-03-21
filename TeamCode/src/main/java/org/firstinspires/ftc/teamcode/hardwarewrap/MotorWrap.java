package org.firstinspires.ftc.teamcode.hardwarewrap;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// wrapper class for motor device
public class MotorWrap {

    // telemetry debugging device
    public Telemetry tele;

    // motor reference
    public Motor motor;

    // device name
    public String name;

    // initialize device
    public MotorWrap(Telemetry tele, HardwareMap map, String name) {

        // telemetry debugging device
        this.tele = tele;

        // debugging data
        tele.addData("init motor start", name);
        tele.update();

        // get motor reference
        motor = new Motor(map, name);
        this.name = name;

        // debugging data
        tele.addData("init motor complete", name);
        tele.update();
    }

    // terminate device
    public void stop() {
        motor.set(0);
    }
}
