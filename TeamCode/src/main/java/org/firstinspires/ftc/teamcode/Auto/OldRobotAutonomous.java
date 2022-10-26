package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.OldRobot;

@Autonomous(name = "Old Robot Autonomous")
public class OldRobotAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        OldRobot robot = new OldRobot(hardwareMap, telemetry, gamepad1);

        robot.backTankDrive.move(1);
        sleep(1000);
        robot.singleJointGripperArm.grab();
        sleep(1000);
        robot.singleJointGripperArm.setArm(430);
        sleep(1000);
        robot.singleJointGripperArm.ungrab();
        sleep(1000);
        robot.singleJointGripperArm.setArm(0);
        sleep(1000);
        robot.backTankDrive.move(-1);
    }
}
