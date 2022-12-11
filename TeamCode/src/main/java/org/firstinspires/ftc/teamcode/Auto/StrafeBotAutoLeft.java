package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.NewRobot;
import org.firstinspires.ftc.teamcode.Robots.OldRobot;
import org.firstinspires.ftc.teamcode.Subsystems.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class StrafeBotAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        NewRobot robot = new NewRobot(this, hardwareMap, telemetry, gamepad1);
    }
}
