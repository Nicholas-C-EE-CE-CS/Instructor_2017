package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Moves the robot for 2 seconds.
 *
 * @author Jochen Fischer
 * @version 2 - uses hardware class for the robot.
 */

@Autonomous(name="Move 2 Seconds V2", group="Test")
//@Disabled
public class Move2Sec extends LinearOpMode {

    // declare robot class:
    HardwareDriveBot robot = new HardwareDriveBot();

    @Override
    public void runOpMode() {

        // initialize the robot:
        robot.init(hardwareMap);

        // wait for the user to press the START button on the DS phone:
        waitForStart();

        // turn the 2 motors on:
        robot.leftMotor.setPower(0.3);
        robot.rightMotor.setPower(0.3);

        // wait 2 seconds
        sleep(2000);

        // turn the 2 motors off:
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);
    }
}
