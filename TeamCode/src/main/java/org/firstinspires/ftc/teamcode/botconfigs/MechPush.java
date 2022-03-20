package org.firstinspires.ftc.teamcode.botconfigs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwarewrap.MechDriveWrap;
import org.firstinspires.ftc.teamcode.hardwarewrap.MotorWrap;

// push bot with mecanum drive train
public class MechPush {

    // telemetry debugging device
    public Telemetry tele;

    // drive train reference
    public MechDriveWrap driveTrain;

    // initialize devices
    public MechPush(Telemetry tele, HardwareMap map) {

        // telemetry debugging devicxe
        this.tele = tele;

        // initialize drive train
        String[] motorNames = {"rf", "lf", "lb", "rb"};
        MotorWrap[] motors = new MotorWrap[motorNames.length];
        for (int i = 0; i < motors.length; i++) {motors[i] = new MotorWrap(tele, map, motorNames[i]);}
        driveTrain = new MechDriveWrap(tele, map, "driveTrain", motors);
    }
}
