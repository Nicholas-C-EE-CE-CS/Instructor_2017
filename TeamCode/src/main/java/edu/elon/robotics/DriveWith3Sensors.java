package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Test program for multiple sensors.
 *
 * The program uses the gamepad to drive the robot.
 * While the robot moves, the light, touch and ultrasonic sensors are read
 * and displayed on the driver station phone continuously.
 *
 * @author Jochen Fischer
 * @version 2 - added light and touch sensor
 * @version 3 - added untrasonic sensor and gamepad bumper to drive slowly.
 */

@TeleOp(name="Drive w/3 Sensors", group="Elon")
// @Disabled
public class DriveWith3Sensors extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // declare robot class:
    HardwareDriveBot robot = new HardwareDriveBot();
    String TAG = "SensorDrive";

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Initialized", "Press START...");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // slow movement is the default, faster with left bumper:
            double factor = robot.SLOW_POWER;
            if (gamepad1.left_bumper) factor = robot.FULL_POWER;

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y * factor;
            double turn  =  gamepad1.right_stick_x * factor;

            telemetry.addLine()
                    .addData("drive", "%.3f", drive)
                    .addData("turn", "%.3f", turn);

            leftPower = drive + turn;   // needs to be limited to [-1,1]
            rightPower = drive - turn;  // needs to be limited to [-1,1]

            leftPower    = Range.clip(leftPower,  -1.0, 1.0) ;
            rightPower   = Range.clip(rightPower, -1.0, 1.0) ;

            // Send calculated power to wheels
            robot.leftMotor.setPower(leftPower);
            robot.rightMotor.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            //--------------------------
            // read the touch sensor
            //--------------------------
            boolean isPressed = robot.touchSensor.isPressed();
            telemetry.addLine().addData("touchSensor", isPressed ? "ON" : "off");

            //--------------------------
            // read the color sensor
            //--------------------------
            int A = robot.colorSensor.alpha();
            int R = robot.colorSensor.red();
            int G = robot.colorSensor.green();
            int B = robot.colorSensor.blue();

            telemetry.addLine()
                    .addData("A", "%3d", A)
                    .addData("R", "%3d", R)
                    .addData("G", "%3d", G)
                    .addData("B", "%3d", B);

            //---------------------------------
            // read the ultrasonic sensor
            // measurements are in cm
            //---------------------------------
            double distance = robot.ultraSonic.getUltrasonicLevel();
            telemetry.addLine().addData("ultraSonic", distance);

            telemetry.update();
        }
    }
}
