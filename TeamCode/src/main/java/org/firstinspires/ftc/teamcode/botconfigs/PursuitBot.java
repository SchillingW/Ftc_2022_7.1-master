package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.function.DoubleSupplier;

// mecanum drive bot with odometry for Pure Pursuit
public class PursuitBot {

    // debugging device
    public Telemetry tele;

    // mecanum wheel drive train
    public MecanumDrive drive;

    // odometry device
    public HolonomicOdometry odometry;

    // robot type data
    public double encoderTicksPerInch = 0;
    public double encoderTrackWidth = 0;
    public double encoderWheelOffset = 0;

    // initialize devices
    public PursuitBot(Telemetry tele, HardwareMap map) {

        // store debugging device
        this.tele = tele;

        // initialize drive train
        drive = new MecanumDrive(
                new Motor(map, "motorFL"),
                new Motor(map, "motorFR"),
                new Motor(map, "motorBL"),
                new Motor(map, "motorBR"));

        // initialize odometry
        odometry = new HolonomicOdometry(
                getSupplier(new Motor(map, "motorFL")),
                getSupplier(new Motor(map, "motorFR")),
                getSupplier(new Motor(map, "motorBL")),
                encoderTrackWidth, encoderWheelOffset);
    }

    // return double supplier representing motor value in inches
    public DoubleSupplier getSupplier(Motor encoder) {

        // convert motor ticks to inches
        return () -> encoder.getCurrentPosition() / encoderTicksPerInch;
    }
}
