package org.firstinspires.ftc.teamcode.old.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.old.Robots.OldRobot;

@Disabled
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
