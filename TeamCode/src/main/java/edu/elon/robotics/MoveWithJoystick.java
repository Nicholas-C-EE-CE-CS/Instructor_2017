package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Moves the robot with a game controller.
 *
 * @author Jochen Fischer
 * @version 2 - uses the robot hardware class
 */

@TeleOp(name="Move with Joystick V2", group="Elon")
// @Disabled
public class MoveWithJoystick extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // declare robot class:
    HardwareDriveBot robot = new HardwareDriveBot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

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
            telemetry.update();
        }
    }
}
