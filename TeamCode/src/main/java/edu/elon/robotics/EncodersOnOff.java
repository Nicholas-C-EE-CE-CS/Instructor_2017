package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turns the motor encoders on and off.
 *
 * After the program is started, the motors will start turning
 * without the uses of encoders. Use the gamepad to turn them on and off:
 *
 * X - turn encoders off
 * Y - turn encoders on
 *
 * Notice the shift in power when the encoders are turned on and off.
 *
 *
 * @author Jochen Fischer
 */

@TeleOp(name="Encoders On Off", group="Elon")
// @Disabled
public class EncodersOnOff extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // declare robot class:
    HardwareDriveBot robot = new HardwareDriveBot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // turn the encoders off to begin with:
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // status display on driver station:
        telemetry.addData("Status", "Initialized, press START...");
        telemetry.addData("Encoders", "off");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run the motors while the program is active:
        robot.leftMotor.setPower(0.4);
        robot.rightMotor.setPower(0.2);

        // state variable for buttons:
        boolean noButtonIsPressed = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // turn encoders off if x button is pressed:
            if (gamepad1.x  &&  noButtonIsPressed) {
                noButtonIsPressed = false;
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Encoders", "off");
                telemetry.update();
            }

            // turn encoders on if y button is pressed:
            if (gamepad1.y  &&  noButtonIsPressed) {
                noButtonIsPressed = false;
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Encoders", "ON");
                telemetry.update();
            }

            // reset button state if both x and y are released:
            if (!gamepad1.x && !gamepad1.y) {
                noButtonIsPressed = true;
            }
        }
    }
}
