package org.firstinspires.ftc.teamcode.old.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.old.Robots.NewRobot;

@Autonomous(name = "New Bot Left Side Auto")
public class StrafeBotAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        NewRobot robot = new NewRobot(this, hardwareMap, telemetry, gamepad1);
        //Put motors in encoder mode
        robot.strafeDrive.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.strafeDrive.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.strafeDrive.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.strafeDrive.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.linearSlide.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //You need to have wait for start or else bad things happen
        waitForStart();
        //1350 is Approximately 25 inches moving forwards
        //1350 is 24 inches moving sideways
        //270 rotation is approximately 35 degrees

        //Directions
        //strafe 39.5 inches
        //place cone
        //strafe 12 inches
        //rotate 180 degrees
        //move forwards 27 inches

//        robot.strafeDrive.strafe(-2221, 0.5f);
//        robot.strafeDrive.drive(162, 162, 0.5f);
//        robot.strafeDrive.drive(-162, -162, 0.5f);
//        robot.strafeDrive.strafe(675, 0.5f);
//        robot.linearSlide.grab();
        robot.strafeDrive.strafein(-39.5, 0.5f);
        robot.strafeDrive.strafe(200, 0.5f);
//        robot.linearSlide.setSlide(100);
//        robot.linearSlide.ungrab();
//        robot.linearSlide.setSlide(0);
        robot.strafeDrive.strafein(-12, 0.5f);
        robot.strafeDrive.drive(-926, 926, 0.5f);
        robot.strafeDrive.strafein(-27, 0.5f);

//        robot.strafeDrive.drive(1680, 1680, 0.5f);
//        robot.strafeDrive.strafe(1000, 0.5f);
//        robot.strafeDrive.strafe(-1000, 0.5f);
//        robot.strafeDrive.drive(1000, -1000, 0.5f);
//        robot.strafeDrive.drive(-520,-520, 0.5f);
//        robot.strafeDrive.drive(-520,-520, -0.5f);

    }
}
