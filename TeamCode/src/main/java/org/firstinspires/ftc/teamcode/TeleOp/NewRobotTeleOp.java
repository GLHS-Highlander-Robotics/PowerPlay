package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.NewRobot;

@TeleOp(name = "Robot TeleOp")
public class NewRobotTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        NewRobot robot = new NewRobot(this, hardwareMap, telemetry, gamepad1);

        robot.strafeDrive.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.strafeDrive.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.strafeDrive.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.strafeDrive.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            robot.strafeDrive.respondToGamepad();
            telemetry.update();
        }
    }
}
