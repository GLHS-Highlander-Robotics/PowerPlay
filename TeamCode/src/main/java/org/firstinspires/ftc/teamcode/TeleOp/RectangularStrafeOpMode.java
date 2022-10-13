package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;


public class RectangularStrafeOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        robot.telemetryBroadcast("Status", "Loaded Strafe OpMode");

        waitForStart();

        robot.telemetryBroadcast("Status", "Starting...");

        while (opModeIsActive()) {
            float speed = -gamepad1.left_stick_x;
            float turn = gamepad1.right_stick_x;
            float strafe = gamepad1.left_stick_x;

            robot.frontLeftMotor.setPower(speed + turn + strafe);
            robot.frontRightMotor.setPower(speed - turn - strafe);
            robot.backLeftMotor.setPower(speed + turn - strafe);
            robot.backRightMotor.setPower(speed - turn + strafe);
        }
    }
}
