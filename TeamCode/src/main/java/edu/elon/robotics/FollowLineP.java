package edu.elon.robotics;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Line follower with proportional controller (P-Controller)
 *
 * @author Jochen Fischer
 */

@Autonomous(name="Follow Line P", group="Elon")
// @Disabled
public class FollowLineP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    String TAG = "FollowLine";

    // declare robot class:
    HardwareDriveBot robot = new HardwareDriveBot();

    // Parameters for the P-controller:
    double speed   = 0.2;
    double maxTurn = 0.1;

    int reference = 26;
    long   dt = 50;            // time interval in milliseconds
    double dT = dt / 1000.0;   // same time interval in seconds
    double Kp = 0.005;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // TODO: calibrate light sensor here:
        int threshold = 26;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // read light sensor:
            int brightness = robot.colorSensor.alpha();

            // implement P-controller:
            double error = reference - brightness;

            // positive "turn" will turn the robot left (CCW):
            double turn = Kp * error;

            // set motors
            double leftPower = speed - turn;
            double rightPower = speed + turn;

            // Send calculated power to wheels
            robot.leftMotor.setPower(leftPower);
            robot.rightMotor.setPower(rightPower);

            // Capture parameters in logcat:
            String message = String.format("%s %2d %2d %5.2f",
                    runtime.toString(), brightness, threshold, turn);
            Log.i(TAG, message);

            // run loop in 50 ms increments:
            robot.waitForTick(dt);
        }
    }
}
