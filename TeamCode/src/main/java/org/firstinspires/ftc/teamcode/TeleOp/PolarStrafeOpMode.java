package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "PolarStrafeOpMode")
public class PolarStrafeOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        robot.telemetryBroadcast("Status", "Loaded Strafe OpMode");

        waitForStart();

        robot.telemetryBroadcast("Status", "Starting...");

        float speedConstant = 0;

        while (opModeIsActive()) {
            float x = gamepad1.left_stick_x;
            float y = -gamepad1.left_stick_y;
            float turn = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double frontLeft = power * cos / max + turn;
            double frontRight = power * sin / max - turn;
            double backLeft = power * sin / max + turn;
            double backRight = power * cos / max - turn;

            if (power + Math.abs(turn) > 1) {
                frontLeft /= power + turn;
                frontRight /= power + turn;
                backLeft /= power + turn;
                backRight /= power + turn;
            }

            frontLeft *= Math.abs(frontLeft) * Math.abs(frontLeft);
            frontRight *= Math.abs(frontRight) * Math.abs(frontRight);
            backLeft *= Math.abs(backLeft) * Math.abs(backLeft);
            backRight *= Math.abs(backRight) * Math.abs(backRight);

            robot.frontLeftMotor.setPower(frontLeft);
            robot.frontRightMotor.setPower(frontRight);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);
        }
    }
}
