package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.NewRobot;

@TeleOp(name = "Robot TeleOp")
public class NewRobotTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        NewRobot robot = new NewRobot(this, hardwareMap, telemetry, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            robot.strafeDrive.respondToGamepad();
            telemetry.update();
        }
    }
}
