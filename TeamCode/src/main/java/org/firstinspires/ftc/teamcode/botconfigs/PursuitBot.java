package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
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

    // hardware specifications
    public double encoderTicksPerRotation = 8192;
    public double wheelDiameter = 2.5;
    public double wheelCircumference = wheelDiameter * Math.PI;

    // robot type data
    public double encoderTicksPerInch = encoderTicksPerRotation / wheelCircumference;
    public double encoderTrackWidth = 20;
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
        Motor encoderL = new Motor(map, "motorFL");
        Motor encoderR = new Motor(map, "motorFR");
        Motor encoderH = new Motor(map, "motorBL");
        odometry = new HolonomicOdometry(
                getSupplier(encoderL),
                getSupplier(encoderR),
                getSupplier(encoderH),
                encoderTrackWidth, encoderWheelOffset);

        // orient to home
        encoderL.resetEncoder();
        encoderR.resetEncoder();
        encoderH.resetEncoder();
        odometry.updatePose();
    }

    // return double supplier representing motor value in inches
    public DoubleSupplier getSupplier(Motor encoder) {

        // convert motor ticks to inches
        return () -> encoder.getCurrentPosition() / encoderTicksPerInch;
    }
}
