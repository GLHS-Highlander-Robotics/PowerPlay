package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.OldRobot;

@TeleOp(name = "Old Robot TeleOp")
public class OldRobotTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        OldRobot robot = new OldRobot(hardwareMap, telemetry, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            robot.backTankDrive.respondToGamepad();
            robot.singleJointGrabberArm.respondToGamepad();
            telemetry.update();
        }
    }
}
