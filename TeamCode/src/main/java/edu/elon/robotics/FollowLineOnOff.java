package edu.elon.robotics;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Line follower with simple on-off controller
 *
 * @author Jochen Fischer
 */

@TeleOp(name="Follow Line On-Off", group="Elon")
// @Disabled
public class FollowLineOnOff extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    String TAG = "FollowLine";

    // declare robot class:
    HardwareDriveBot robot = new HardwareDriveBot();

    // Parameters for the controller:
    double speed   = 0.2;
    double maxTurn = 0.1;

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

            // positive "turn" will turn the robot left (CCW):
            double turn;
            if (brightness < threshold)  turn = maxTurn;
            else                         turn = -maxTurn;


            double leftPower = speed - turn;
            double rightPower = speed + turn;

            // Send calculated power to wheels
            robot.leftMotor.setPower(leftPower);
            robot.rightMotor.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            // Capture parameters in logcat:
            String message = String.format("%s %2d %2d %5.2f",
                    runtime.toString(), brightness, threshold, turn);
            Log.i(TAG, message);

            // run loop in 50 ms increments:
            robot.waitForTick(50);
        }
    }
}
