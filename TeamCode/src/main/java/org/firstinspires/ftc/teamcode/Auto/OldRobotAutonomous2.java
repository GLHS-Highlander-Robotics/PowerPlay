package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.OldRobot;
import org.firstinspires.ftc.teamcode.Util.Maths;

@Autonomous(name = "Old Robot Autonomous")
public class OldRobotAutonomous2 extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;
    private DcMotor armMotor;
    private int leftPos2 = 0;
    private int rightPos2 = 0;
    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "b_left");
        right = hardwareMap.get(DcMotor.class, "b_right");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        //Put motors in encoder mode
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setDirection(DcMotor.Direction.REVERSE);

        //You need to have wait for start or else bad things happen
        waitForStart();

            drive2(1000, 1000, 0.25f);
            drive2(500, -500, 0.25f);
            drive2(200, 200, 0.10f);
            drive2(-200, -200, 0.10f);
            drive2(-500, 500, 0.25f);
            drive2(1000, 1000, 0.75f);


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
    private void drive2(int leftmove, int rightmove, float speed) {
        leftPos2 += leftmove;
        rightPos2 += rightmove;
        left.setTargetPosition(leftPos2);
        right.setTargetPosition(rightPos2);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(speed);
        right.setPower(speed);

        while (opModeIsActive() && left.isBusy() && right.isBusy()) {
            idle();
        }
    }


}
