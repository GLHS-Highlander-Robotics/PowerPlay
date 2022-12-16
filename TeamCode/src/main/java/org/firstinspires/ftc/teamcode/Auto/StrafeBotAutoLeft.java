package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.NewRobot;
import org.firstinspires.ftc.teamcode.Robots.OldRobot;
import org.firstinspires.ftc.teamcode.Util.Measure;
import org.firstinspires.ftc.teamcode.Subsystems.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "New Bot Left")
public class StrafeBotAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        NewRobot robot = new NewRobot(this, hardwareMap, telemetry, gamepad1);
        //Put motors in encoder mode
        robot.strafeDrive.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.strafeDrive.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.strafeDrive.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.strafeDrive.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //You need to have wait for start or else bad things happen
        waitForStart();
        //1350 is Approximately 25 inches moving forwards
        //1350 is 24 inches moving sideways
        //270 rotation is approximately 35 degrees

        //Directions
        //strafe 39.5 inches
        //strafe 12 inches
        //rotate 180 degrees
        //move forwards 27 inches

//        robot.strafeDrive.strafe(-2221, 0.5f);
//        robot.strafeDrive.drive(162, 162, 0.5f);
//        robot.strafeDrive.drive(-162, -162, 0.5f);
//        robot.strafeDrive.strafe(675, 0.5f);
        robot.strafeDrive.drive(-1750, 1750, 0.5f);
        robot.strafeDrive.strafein(-39.5, 0.5f);



//        robot.strafeDrive.drive(1680, 1680, 0.5f);
//        robot.strafeDrive.strafe(1000, 0.5f);
//        robot.strafeDrive.strafe(-1000, 0.5f);
//        robot.strafeDrive.drive(1000, -1000, 0.5f);
//        robot.strafeDrive.drive(-520,-520, 0.5f);
//        robot.strafeDrive.drive(-520,-520, -0.5f);

    }
}
