package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robots.OldRobot;
import org.firstinspires.ftc.teamcode.Util.Maths;

@Autonomous(name = "Better Robot Autonomous")
public class BetterRobotAutonomous extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;
    private DcMotor arm;
    private Servo grip1;
    private int lPos;
    private int rPos;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "b_left");
        right = hardwareMap.get(DcMotor.class, "b_right");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grip1 = hardwareMap.get(Servo.class, "grip1");

        //Put motors in encoder mode
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        right.setDirection(DcMotorSimple.Direction.REVERSE);
        //You need to have wait for start or else bad things happen
        waitForStart();

        move(2080, 2080, 0.50);
        armSet(89);
        move(520, -520, 0.50);




    }
    private void move(int leftTarget, int rightTarget, double speed) {
        lPos += leftTarget;
        rPos += rightTarget;

        left.setTargetPosition(lPos);
        right.setTargetPosition(rPos);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(speed);
        right.setPower(speed);

        while(opModeIsActive() && left.isBusy() && right.isBusy()) {
            idle();
        }
    }

    private void armSet(int steps) {
        arm.setTargetPosition(Maths.clamp(steps, 0, 450));
        while (opModeIsActive() && arm.isBusy()) {
            idle();
        }
    }

}
