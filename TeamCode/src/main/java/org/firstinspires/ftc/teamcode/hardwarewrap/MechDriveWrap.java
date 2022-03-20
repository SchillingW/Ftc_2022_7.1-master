package org.firstinspires.ftc.teamcode.hardwarewrap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// wrapper class for mecanum drive train
public class MechDriveWrap {

    // telemetry debugging device
    public Telemetry tele;

    // drive train reference
    public MecanumDrive driveTrain;

    // motor references
    public MotorWrap[] motors;

    // device name
    public String name;

    // initialize device
    public MechDriveWrap(Telemetry tele, HardwareMap map, String name, MotorWrap[] motors) {

        // telemetry debugging device
        this.tele = tele;

        // debugging data
        tele.addData("init mech drive start", name);
        tele.update();

        // get drive train reference
        driveTrain = new MecanumDrive(motors[1].motor, motors[0].motor, motors[2].motor, motors[3].motor);
        this.motors = motors;
        this.name = name;

        // debugging data
        tele.addData("init mech drive complete", name);
        tele.update();
    }
}
