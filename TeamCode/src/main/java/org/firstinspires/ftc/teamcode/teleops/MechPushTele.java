package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.botconfigs.MechPush;

// TeleOp for MechPush
@TeleOp(name="MechPushTele", group="Ftc2022")
public class MechPushTele extends OpMode {

    // robot reference
    public MechPush robot;

    // gamepad references
    public GamepadEx pad1;

    // runs on initialization
    @Override
    public void init() {
        robot = new MechPush(telemetry, hardwareMap);
        pad1 = new GamepadEx(gamepad1);
    }

    // runs every frame after play
    @Override
    public void loop() {

        // run drive train
        robot.driveTrain.driveTrain.driveRobotCentric(pad1.getLeftX(), pad1.getLeftY(), pad1.getRightX());
    }
}
