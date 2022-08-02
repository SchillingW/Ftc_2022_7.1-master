package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import java.util.ArrayList;

// pure pursuit algorithm demo
@TeleOp(name="PursuitBotDemo", group="PursuitBot")
public class PursuitBotDemo extends LinearOpMode {

    // robot reference
    public PursuitBot robot;

    // robot poses which form recorded path
    public ArrayList<Pose2d> recording;

    // movement parameters
    public double movementSpeed = 0;
    public double turnSpeed = 0;
    public double followRadius = 0;
    public double positionBuffer = 0;
    public double rotationBuffer = 0;

    @Override
    public void runOpMode() {

        // get reference to robot
        robot = new PursuitBot(telemetry, hardwareMap);

        // wait for user to start program
        waitForStart();

        // keep looping while program is running
        while (opModeIsActive()) {

            // loop through demo states
            RecordPath();
            ReturnHome();
            FollowPath();
            ReturnHome();
        }
    }

    // behaviour for robot driving and user path recording
    public void RecordPath() {

        // reset recorded poses
        recording = new ArrayList<>();
        boolean recordInputLast = gamepad1.b;

        // keep looping while program is running and a is not pressed
        while (opModeIsActive() && !gamepad1.a) {

            // drive based on
            robot.drive.driveRobotCentric(
                    gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // add current pose to recording if b pressed
            boolean recordInputNew = gamepad1.b;
            if (recordInputNew && !recordInputLast) recording.add(robot.odometry.getPose());
            recordInputLast = recordInputNew;
        }

        // stop drive train
        robot.drive.stop();
    }

    // follows recorded path with pure pursuit
    public void FollowPath() {

        // check that program is running
        if (opModeIsActive()) {

            // create start and end waypoints from current pose to last pose in recording
            Waypoint[] points = new Waypoint[recording.size() + 1];
            points[0] = new StartWaypoint(robot.odometry.getPose());
            points[points.length - 1] = new EndWaypoint(recording.get(recording.size() - 1),
                    movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

            // iterate through recorded poses and convert to waypoints
            for (int i = 0; i < recording.size() - 1; i++) {
                points[i + 1] = new GeneralWaypoint(recording.get(i),
                        movementSpeed, turnSpeed, followRadius);
            }

            // follow path formed by waypoints
            Path path = new Path(points);
            path.init();
            path.followPath(robot.drive, robot.odometry);

            // wait at home for a second
            sleep(1000);
        }
    }

    // returns to origin position with pure pursuit
    public void ReturnHome() {

        // check that program is running
        if (opModeIsActive()) {

            // create start and end waypoints from current pose to origin pose
            Waypoint start = new StartWaypoint(robot.odometry.getPose());
            Waypoint end = new EndWaypoint(new Pose2d(),
                    movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

            // follow path formed by waypoints
            Path path = new Path(start, end);
            path.init();
            path.followPath(robot.drive, robot.odometry);

            // wait at home for a second
            sleep(1000);
        }
    }
}
