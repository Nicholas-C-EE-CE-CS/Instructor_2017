package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 *

 */

@Autonomous(name="Move 2 Seconds", group="Test")
//@Disabled
public class Move2Sec extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    @Override
    public void runOpMode() {

        // map motors to hardware
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // wait for the user to press the START button on the DS phone:
        waitForStart();

        // turn the 2 motors on:
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        // wait 2 seconds
        sleep(2000);

        // turn the 2 motors off:
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

    }
}
