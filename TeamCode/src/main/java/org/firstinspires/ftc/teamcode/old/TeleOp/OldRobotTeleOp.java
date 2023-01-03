package org.firstinspires.ftc.teamcode.old.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.old.Robots.OldRobot;

@TeleOp(name = "Old Robot TeleOp")
public class OldRobotTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        OldRobot robot = new OldRobot(this, hardwareMap, telemetry, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            robot.backTankDrive.respondToGamepad();
            robot.singleJointGripperArm.respondToGamepad();
            telemetry.update();
        }
    }
}
