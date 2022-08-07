package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HoloOdomFlip;

import java.util.function.DoubleSupplier;

// mecanum drive bot with odometry for Pure Pursuit
public class PursuitBot {

    // debugging device
    public  Telemetry tele;

    // mecanum wheel drive train
    public MecanumDrive drive;
    public Motor motorFL;
    public Motor motorFR;
    public Motor motorBL;
    public Motor motorBR;

    // odometry device
    public OdometrySubsystem odometry;
    public DoubleSupplier encoderL;
    public DoubleSupplier encoderR;
    public DoubleSupplier encoderH;

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
        motorFL = new Motor(map, "motorFL");
        motorFR = new Motor(map, "motorFR");
        motorBL = new Motor(map, "motorBL");
        motorBR = new Motor(map, "motorBR");
        drive = new MecanumDrive(motorFL, motorFR, motorBL, motorBR);

        // initialize odometry
        encoderL = getSupplier(motorFL, -1);
        encoderR = getSupplier(motorFR, 1);
        encoderH = getSupplier(motorBL, -1);
        odometry = new OdometrySubsystem(new HoloOdomFlip(
                encoderL, encoderR, encoderH,
                encoderTrackWidth, encoderWheelOffset));

        // orient to home
        motorFL.resetEncoder();
        motorFR.resetEncoder();
        motorBL.resetEncoder();
        motorBR.resetEncoder();
    }

    // return double supplier representing motor value in inches
    public DoubleSupplier getSupplier(Motor encoder, float coefficient) {

        // convert motor ticks to inches
        return () -> encoder.getCurrentPosition() / encoderTicksPerInch * coefficient;
    }
}
