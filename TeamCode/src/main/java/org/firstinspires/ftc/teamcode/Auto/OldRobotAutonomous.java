package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.OldRobot;

@Autonomous(name = "Old Robot Autonomous")
public class OldRobotAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        OldRobot robot = new OldRobot(this, hardwareMap, telemetry, gamepad1);

        //Put motors in encoder mode
        robot.backTankDrive.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backTankDrive.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.singleJointGripperArm.ungrab();
        //You need to have wait for start or else bad things happen
        waitForStart();

        if (opModeIsActive()) {
            robot.singleJointGripperArm.grab();
//            robot.backTankDrive.drive(-1000, -1000, 0.25f);
//            robot.backTankDrive.drive(-500, 500, 0.25f);
//            robot.singleJointGripperArm.setArm(89);
//            robot.backTankDrive.drive(-200, -200, 0.10f);
//            robot.singleJointGripperArm.ungrab();
//            robot.backTankDrive.drive(200, 200, 0.10f);
//            robot.singleJointGripperArm.setArm(0);
//            robot.backTankDrive.drive(500, -500, 0.25f);
//            robot.backTankDrive.drive(-1000, -1000, 0.75f);
        }


//         robot.backTankDrive.move(1);
//         sleep(1000);
//         robot.singleJointGripperArm.grab();
//         sleep(1000);
//         robot.singleJointGripperArm.setArm(430);
//         sleep(1000);
//         robot.singleJointGripperArm.ungrab();
//         sleep(1000);
//         robot.singleJointGripperArm.setArm(0);
//         sleep(1000);
//         robot.backTankDrive.move(-1);
    }


}
