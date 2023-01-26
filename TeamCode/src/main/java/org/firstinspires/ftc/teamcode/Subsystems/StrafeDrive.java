package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;

public class StrafeDrive implements Subsystem {
    private final RobotOpMode opMode;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private int flPos = 0;
    private int frPos = 0;
    private int blPos = 0;
    private int brPos = 0;
    private double limiter = 0.75;
    private boolean slow = false;

    public StrafeDrive(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void setup() {
        frontLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_left");
        frontRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_right");
        backLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_left");
        backRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_right");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(int leftMove, int rightMove, float speed) {

        flPos += leftMove;
        frPos += rightMove;
        blPos += leftMove;
        brPos += rightMove;

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void driveInches(double leftInches, double rightInches, float speed) {

        flPos += Utils.driveTicks(leftInches);
        frPos += Utils.driveTicks(rightInches);
        blPos += Utils.driveTicks(leftInches);
        brPos += Utils.driveTicks(rightInches);

        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);
        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);


        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
//        while (opMode.opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
//            opMode.idle();
//        }
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }


    public void strafe(int move, float speed) {

        flPos = move;
        frPos = move;
        blPos -= move;
        brPos -= move;

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void strafeInches(double inches, float speed) {

        flPos += Utils.strafeTicks(inches);
        frPos += Utils.strafeTicks(inches);
        blPos -= Utils.strafeTicks(inches);
        brPos -= Utils.strafeTicks(inches);
        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void updateByTwoGamepads(){
        double forward;
        double strafe;
        double rotate;
//        if (Math.abs(-opMode.gamepad2.left_stick_y) >= 0.01 || Math.abs(opMode.gamepad1.left_stick_x) >= 0.01 || Math.abs(opMode.gamepad1.right_stick_x) >= 0.01) {
//
//            slow = true;
//        } else {
//            slow = false;
//        }
        if(opMode.gamepad1.a){
            limiter = 0.25;
        }else if(opMode.gamepad1.b){
            limiter = 0.75;
        }
//        if (slow) {
//            forward = -opMode.gamepad2.left_stick_y * (limiter / 2);
//            strafe = opMode.gamepad2.left_stick_x * (limiter / 2);
//            rotate = opMode.gamepad2.right_stick_x * (limiter / 2);
//            if (Math.abs(-opMode.gamepad2.left_stick_y) < 0.01) {
//                forward = 0;
//            }
//            if (Math.abs(opMode.gamepad2.left_stick_x) < 0.01) {
//                strafe = 0;
//            }
//            if (Math.abs(opMode.gamepad2.right_stick_x) < 0.01) {
//                rotate = 0;
//            }
//        } else{
            forward = -opMode.gamepad1.left_stick_y * limiter;
            strafe = opMode.gamepad1.left_stick_x * limiter;
            rotate = opMode.gamepad1.right_stick_x * limiter;
            if (Math.abs(-opMode.gamepad1.left_stick_y) < 0.01) {
                forward = 0;
            }
            if (Math.abs(opMode.gamepad1.left_stick_x) < 0.01) {
                strafe = 0;
            }
            if (Math.abs(opMode.gamepad1.right_stick_x) < 0.01) {
                rotate = 0;
                rotate = opMode.gamepad2.right_stick_x * 0.25;
                if(Math.abs(opMode.gamepad2.right_stick_x)<0.02){
                    rotate=0;
                }
            }
//        }
        frontLeftMotor.setPower((forward + strafe + rotate));
        backLeftMotor.setPower((forward - strafe + rotate));
        frontRightMotor.setPower((forward + strafe - rotate));
        backRightMotor.setPower((forward - strafe - rotate));
        opMode.telemetry.addData("Limiter: ", limiter);
        opMode.telemetry.update();
    }

    public void updateByGamepad() {
        double forward;
        double strafe;
        double rotate;
//        if (Math.abs(-opMode.gamepad2.left_stick_y) >= 0.01 || Math.abs(opMode.gamepad1.left_stick_x) >= 0.01 || Math.abs(opMode.gamepad1.right_stick_x) >= 0.01) {
//            slow = true;
//        } else {
//            slow = false;
//        }
//        if (slow) {
//            forward = -opMode.gamepad2.left_stick_y * (limiter / 2);
//            strafe = opMode.gamepad2.left_stick_x * (limiter / 2);
//            rotate = opMode.gamepad2.right_stick_x * (limiter / 2);
//            if (Math.abs(-opMode.gamepad2.left_stick_y) < 0.01) {
//                forward = 0;
//            }
//            if (Math.abs(opMode.gamepad2.left_stick_x) < 0.01) {
//                strafe = 0;
//            }
//            if (Math.abs(opMode.gamepad2.right_stick_x) < 0.01) {
//                rotate = 0;
//            }
//        } else {
            forward = -opMode.gamepad1.left_stick_y * limiter;
            strafe = opMode.gamepad1.left_stick_x * limiter;
            rotate = opMode.gamepad1.right_stick_x * limiter;
            if (Math.abs(-opMode.gamepad1.left_stick_y) < 0.01) {
                forward = 0;
            }
            if (Math.abs(opMode.gamepad1.left_stick_x) < 0.01) {
                strafe = 0;
            }
            if (Math.abs(opMode.gamepad1.right_stick_x) < 0.01) {
                rotate = 0;
            }

        frontLeftMotor.setPower((forward + strafe + rotate));
        backLeftMotor.setPower((forward - strafe + rotate));
        frontRightMotor.setPower((forward + strafe - rotate));
        backRightMotor.setPower((forward - strafe - rotate));
        opMode.telemetry.addData("Limiter: ", limiter);
        opMode.telemetry.update();
    }

    public void setModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    public void setPowers(float speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }
}
