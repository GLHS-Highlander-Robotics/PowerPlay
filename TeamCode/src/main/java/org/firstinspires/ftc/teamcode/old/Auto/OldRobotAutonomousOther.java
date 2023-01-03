package org.firstinspires.ftc.teamcode.old.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.old.Robots.OldRobot;

@Disabled
public class OldRobotAutonomousOther extends LinearOpMode {
    @Override
    public void runOpMode() {
        OldRobot robot = new OldRobot(this, hardwareMap, telemetry, gamepad1);

        //Put motors in encoder mode
        robot.backTankDrive.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backTankDrive.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.singleJointGripperArm.grab();
        //You need to have wait for start or else bad things happen
        waitForStart();

        //2080 ticks is 2 rotations
//        robot.singleJointGripperArm.ungrab();
//        robot.backTankDrive.drive(-400, -400, 0.5f);
//        robot.backTankDrive.drive(520, -520, 0.5f);
//        robot.singleJointGripperArm.setArm(290);
//        robot.backTankDrive.drive(-200, -200, 0.25f);
//        robot.singleJointGripperArm.grab();
//        robot.backTankDrive.drive(520, 520, 0.25f);
//        robot.singleJointGripperArm.setArm(0);
//        robot.backTankDrive.drive(-375, 375, 0.5f);
//        robot.backTankDrive.drive(-3120, -3120, 0.5f);
//          IMPORTANT: TO GO FORWARD PUT NEGATIVE VALUES

//Mid Height Pole
        robot.singleJointGripperArm.ungrab();
        robot.backTankDrive.drive(-3120, -3120, 0.5f);
        robot.singleJointGripperArm.setArm(450);
        robot.backTankDrive.drive(520, -520, 0.5f);
        robot.backTankDrive.drive(-400, -400, 0.25f);
        robot.singleJointGripperArm.grab();
        robot.backTankDrive.drive(520, 520, 0.25f);
        robot.singleJointGripperArm.setArm(0);
        robot.backTankDrive.drive(-375, 375, 0.5f);
        robot.backTankDrive.drive(-400, -400, 0.5f);
    }


}

